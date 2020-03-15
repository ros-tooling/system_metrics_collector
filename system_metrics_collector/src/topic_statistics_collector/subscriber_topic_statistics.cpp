// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rcl/time.h"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

#include "received_message_age.hpp"
#include "received_message_period.hpp"
#include "subscriber_topic_statistics.hpp"
#include "system_metrics_collector/constants.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;

namespace topic_statistics_collector
{
static rcl_interfaces::msg::IntegerRange buildPositiveIntegerRange(
  int64_t from,
  int64_t to,
  uint64_t step)
{
  rcl_interfaces::msg::IntegerRange positive_range;
  positive_range.from_value = from;
  positive_range.to_value = to;
  positive_range.step = step;

  return positive_range;
}

static rcl_interfaces::msg::ParameterDescriptor buildPeriodParameterDescriptor(
  const std::string & description)
{
  const auto positive_range = buildPositiveIntegerRange(
    1,
    std::numeric_limits<decltype(rcl_interfaces::msg::IntegerRange::to_value)>::max(),
    1
  );

  rcl_interfaces::msg::ParameterDescriptor period_descriptor;
  period_descriptor.read_only = true;
  period_descriptor.integer_range.push_back(positive_range);
  period_descriptor.description = description;

  return period_descriptor;
}

static rcl_interfaces::msg::ParameterDescriptor buildTopicParameterDescriptor(
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor topic_descriptor;
  topic_descriptor.read_only = true;

  return topic_descriptor;
}

static void validateStringParam(const std::string & param)
{
  if (param.empty()) {
    std::stringstream ss;
    ss << param << " node paramater cannot be empty";
    throw std::invalid_argument{ss.str().c_str()};
  }
}

static void initializeClock(rcl_clock_t & clock_)
{
  auto allocator = rcl_get_default_allocator();
  if (RCL_RET_OK != rcl_clock_init(RCL_STEADY_TIME, &clock_, &allocator)) {
    throw std::runtime_error{"Failed to initialize clock"};
  }
}

static rcl_time_point_value_t getCurrentTime(rcl_clock_t & clock)
{
  rcl_time_point_value_t now_time_point;
  if (RCL_RET_OK != rcl_clock_get_now(&clock, &now_time_point)) {
    RCUTILS_LOG_ERROR_NAMED("SubscriberTopicStatistics", "Failed to get current time");
    return 0;
  }
  return now_time_point;
}

template<typename T>
static void initializeCollectors(
  std::vector<std::shared_ptr<TopicStatisticsCollector<T>>> & container)
{
  container.push_back(std::make_shared<ReceivedMessageAgeCollector<T>>());
  container.push_back(std::make_shared<ReceivedMessagePeriodCollector<T>>());
}

template<typename T>
SubscriberTopicStatisticsNode<T>::SubscriberTopicStatisticsNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode{node_name, node_options}
{
  const auto publish_param_desc = buildPeriodParameterDescriptor(
    "The period in milliseconds between each published MetricsMessage."
  );
  auto publish_period = declare_parameter(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    system_metrics_collector::collector_node_constants::kDefaultPublishPeriod.count(),
    publish_param_desc);
  publish_period_ = std::chrono::milliseconds{publish_period};

  const auto collect_topic_desc = buildTopicParameterDescriptor(
    "The topic to subscribe to, for calculating topic statistics.");
  collect_topic_name_ = declare_parameter(
    topic_statistics_constants::kCollectStatsTopicName,
    topic_statistics_constants::kDefaultCollectStatsTopicName,
    collect_topic_desc);
  validateStringParam(collect_topic_name_);

  const auto publish_topic_desc = buildTopicParameterDescriptor(
    "The topic to publish topic statistics to.");
  publish_topic_name_ = declare_parameter(
    topic_statistics_constants::kPublishStatsTopicName,
    system_metrics_collector::collector_node_constants::kStatisticsTopicName,
    publish_topic_desc);
  validateStringParam(publish_topic_name_);

  initializeClock(clock_);

  initializeCollectors(statistics_collectors_);

  auto callback = [this](typename T::SharedPtr received_message) {
      for (const auto & collector : statistics_collectors_) {
        collector->OnMessageReceived(
          *received_message,
          getCurrentTime(clock_));
      }
    };
  subscription_ = create_subscription<T>(collect_topic_name_, 10, callback);
}

template<typename T>
SubscriberTopicStatisticsNode<T>::~SubscriberTopicStatisticsNode()
{
  if (RCL_RET_OK != rcl_clock_fini(&clock_)) {
    RCUTILS_LOG_WARN("Failed to finalize clock");
  }
}

template<typename T>
void SubscriberTopicStatisticsNode<T>::StartPublisher()
{
  rcpputils::check_true(publish_timer_ == nullptr);

  if (publisher_ == nullptr) {
    publisher_ = create_publisher<MetricsMessage>(publish_topic_name_, 10);
  }

  publisher_->on_activate();

  RCLCPP_DEBUG(this->get_logger(), "SetupStart: creating publish_timer_");
  publish_timer_ = this->create_wall_timer(
    publish_period_, [this]() {
      this->PublishStatisticMessage();
      this->ClearCollectorMeasurements();
      this->window_start_ = this->now();
    });
  window_start_ = now();
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatisticsNode<T>::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_activate");
  for (const auto & collector : statistics_collectors_) {
    if (!collector->Start()) {
      return CallbackReturn::ERROR;
    }
  }

  StartPublisher();
  return CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatisticsNode<T>::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
  for (const auto & collector : statistics_collectors_) {
    if (!collector->Stop()) {
      return CallbackReturn::ERROR;
    }
  }

  StopPublisher();
  return CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatisticsNode<T>::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_shutdown");
  publisher_.reset();

  return CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatisticsNode<T>::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_error");
  if (publisher_ != nullptr) {
    publisher_.reset();
  }
  return CallbackReturn::SUCCESS;
}

template<typename T>
void SubscriberTopicStatisticsNode<T>::ClearCollectorMeasurements()
{
  for (const auto & collector : statistics_collectors_) {
    collector->ClearCurrentMeasurements();
  }
}

template<typename T>
void SubscriberTopicStatisticsNode<T>::StopPublisher()
{
  rcpputils::check_true(publish_timer_ != nullptr);
  rcpputils::check_true(publisher_ != nullptr);

  publisher_->on_deactivate();

  publish_timer_->cancel();
  publish_timer_.reset();
}

template<typename T>
void SubscriberTopicStatisticsNode<T>::PublishStatisticMessage()
{
  rcpputils::check_true(publisher_ != nullptr);
  rcpputils::check_true(publisher_->is_activated());

  for (const auto & collector : statistics_collectors_) {
    const auto msg = system_metrics_collector::GenerateStatisticMessage(
      get_name(),
      collector->GetMetricName(),
      collector->GetMetricUnit(),
      window_start_,
      now(),
      collector->GetStatisticsResults());
    publisher_->publish(msg);
  }
}

}  // namespace topic_statistics_collector
