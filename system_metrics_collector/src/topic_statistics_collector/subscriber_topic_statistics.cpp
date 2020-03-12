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
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "subscriber_topic_statistics.hpp"
#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/metrics_message_publisher.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using std::placeholders::_1;

namespace topic_statistics_collector
{
template<typename T>
SubscriberTopicStatistics<T>::SubscriberTopicStatistics(
  const std::string & node_name,
  const rclcpp::NodeOptions & node_options,
  const std::string & topic_name)
: rclcpp_lifecycle::LifecycleNode{node_name, node_options},
  topic_name_(topic_name)
{
  rcl_interfaces::msg::IntegerRange positive_range;
  positive_range.from_value = 1;
  positive_range.to_value =
    std::numeric_limits<decltype(rcl_interfaces::msg::IntegerRange::to_value)>::max();
  positive_range.step = 1;

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.integer_range.push_back(positive_range);

  descriptor.description =
    "The period in milliseconds between each published MetricsMessage.";
  auto publish_period = declare_parameter(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    system_metrics_collector::collector_node_constants::kDefaultPublishPeriod.count(),
    descriptor);
  publish_period_ = std::chrono::milliseconds{publish_period};

  statistics_collectors_.push_back(std::make_unique<ReceivedMessageAgeCollector<T>>());
  statistics_collectors_.push_back(std::make_unique<ReceivedMessagePeriodCollector<T>>());

  auto qos_options_ = rclcpp::QoS(rclcpp::KeepAll());
  subscription_ = create_subscription<T>(
    topic_name_,
    qos_options_,
    std::bind(&SubscriberTopicStatistics::CollectorCallback, this, _1),
    subscription_options_);
}

template<typename T>
bool SubscriberTopicStatistics<T>::SetupStart()
{
  assert(publish_timer_ == nullptr);

  if (publisher_ == nullptr) {
    publisher_ = create_publisher<MetricsMessage>(
      system_metrics_collector::collector_node_constants::kStatisticsTopicName,
      10 /*history_depth*/);
  }

  publisher_->on_activate();

  RCLCPP_DEBUG(this->get_logger(), "SetupStart: creating publish_timer_");
  publish_timer_ = this->create_wall_timer(
    publish_period_, [this]() {
      this->PublishStatisticMessage();
      this->ClearCurrentMeasurements();
      this->window_start_ = this->now();
    });

  window_start_ = now();

  return true;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatistics<T>::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_DEBUG(this->get_logger(), "on_activate");
  const auto ret = Start();
  return ret ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatistics<T>::on_deactivate(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
  const auto ret = Stop();
  return ret ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatistics<T>::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_shutdown");
  Stop();
  publisher_.reset();
  return CallbackReturn::SUCCESS;
}

template<typename T>
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SubscriberTopicStatistics<T>::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_DEBUG(this->get_logger(), "on_error");
  Stop();
  if (publisher_) {
    publisher_.reset();
  }
  return CallbackReturn::SUCCESS;
}

template<typename T>
bool SubscriberTopicStatistics<T>::SetupStop()
{
  assert(publish_timer_ != nullptr);
  assert(publisher_ != nullptr);

  publisher_->on_deactivate();

  publish_timer_->cancel();
  publish_timer_.reset();

  return true;
}

template<typename T>
void SubscriberTopicStatistics<T>::CollectorCallback(const typename T::SharedPtr received_message)
{
  for (const auto & collector : statistics_collectors_) {
    collector->OnMessageReceived(*received_message);
  }
}

template<typename T>
std::string SubscriberTopicStatistics<T>::GetStatusString() const
{
  std::stringstream ss;
  ss << "name=" << get_name() <<
    ", publishing_topic=" << (publisher_ ? publisher_->get_topic_name() : "") <<
    ", publish_period=" << std::to_string(publish_period_.count()) + "ms" <<
    ", " << Collector::GetStatusString();
  return ss.str();
}

template<typename T>
void SubscriberTopicStatistics<T>::PublishStatisticMessage()
{
  assert(publisher_ != nullptr);
  assert(publisher_->is_activated());

  const auto msg = system_metrics_collector::GenerateStatisticMessage(
    get_name(),
    GetMetricName(),
    GetMetricUnit(),
    window_start_,
    now(),
    GetStatisticsResults());
  publisher_->publish(msg);
}

}  // namespace topic_statistics_collector
