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

#ifndef TOPIC_STATISTICS_COLLECTOR__SUBSCRIBER_TOPIC_STATISTICS_HPP_
#define TOPIC_STATISTICS_COLLECTOR__SUBSCRIBER_TOPIC_STATISTICS_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/asserts.hpp"

#include "libstatistics_collector/collector/generate_statistics_message.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_age.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_period.hpp"
#include "libstatistics_collector/topic_statistics_collector/topic_statistics_collector.hpp"

#include "parameter_utils.hpp"
#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/metrics_message_publisher.hpp"

namespace topic_statistics_collector
{
using libstatistics_collector::topic_statistics_collector::TopicStatisticsCollector;
using libstatistics_collector::topic_statistics_collector::ReceivedMessageAgeCollector;
using libstatistics_collector::topic_statistics_collector::ReceivedMessagePeriodCollector;
namespace constants =
  libstatistics_collector::topic_statistics_collector::topic_statistics_constants;

/**
 * Class which makes periodic topic statistics measurements and publishes them,
 * using a ROS2 timer.
 *
 * @tparam T the ROS2 message type to observe and collect statistics
 */
template<typename T>
class SubscriberTopicStatisticsNode : public system_metrics_collector::MetricsMessagePublisher,
  public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Constructs a SubscriberTopicStatisticsNode node.
   * The following parameter can be set via the rclcpp::NodeOptions:
   * `publish_period`: the period at which metrics are published
   * `collect_topic_names`: the topics to subscribe to and collect statistics
   * `publish_topic_name`: the topic to publish collected statistics to
   *
   * @param node_name the name of this node, it must be non-empty
   * @param node_options the options (arguments, parameters, etc.) for this node
   * @param topic_name the name of topic to compute statistics for
   */
  SubscriberTopicStatisticsNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options)
  : rclcpp_lifecycle::LifecycleNode{node_name, node_options}
  {
    const auto publish_param_desc = BuildPeriodParameterDescriptor(
      "The period in milliseconds between each published MetricsMessage."
    );
    const auto publish_period = declare_parameter(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      system_metrics_collector::collector_node_constants::kDefaultPublishPeriod.count(),
      publish_param_desc);
    publish_period_ = std::chrono::milliseconds{publish_period};

    const auto collect_topic_desc = BuildTopicParameterDescriptor(
      "The topic to subscribe to, for calculating topic statistics.");
    collect_topic_names_ = declare_parameter(
      constants::kCollectStatsTopicNameParam,
      std::vector<std::string>() /* setting default to empty, since this is a required parameter */,
      collect_topic_desc);

    rcpputils::require_true(
      !collect_topic_names_.empty(),
      "The topic name vector cannot be empty");

    const auto publish_topic_desc = BuildTopicParameterDescriptor(
      "The topic to publish topic statistics to.");
    publish_topic_name_ = declare_parameter(
      constants::kPublishStatsTopicNameParam,
      system_metrics_collector::collector_node_constants::kStatisticsTopicName,
      publish_topic_desc);

    InitializeCollectors();

    auto callback = [this](typename T::SharedPtr received_message) {
        for (const auto & collector : statistics_collectors_) {
          RCLCPP_DEBUG(this->get_logger(), collector->GetStatusString());
          collector->OnMessageReceived(*received_message, this->LifecycleNode::now().nanoseconds());
        }
      };

    // Create a publisher with QoS histor_depth set to 10.
    for (const auto & topic_name_str : collect_topic_names_) {
      ValidateStringParam(
        constants::kCollectStatsTopicNameParam,
        topic_name_str);
      subscriptions_.push_back(create_subscription<T>(topic_name_str, 10, callback));
    }
  }

  virtual ~SubscriberTopicStatisticsNode() = default;

  /**
   * Starts the node.
   *
   * @param state input state unused
   * @return CallbackReturn success if start returns true, error otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state)
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

  /**
   * Stops the node.
   *
   * @param input state unused
   * @return CallbackReturn success if start returns true, error otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_deactivate");
    for (const auto & collector : statistics_collectors_) {
      if (!collector->Stop()) {
        return CallbackReturn::ERROR;
      }
    }

    publish_timer_->cancel();
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  /**
   * Stops the node and performs cleanup.
   *
   * @param input state unused
   * @return CallbackReturn success
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_shutdown");
    StopPublisher();
    publisher_.reset();

    return CallbackReturn::SUCCESS;
  }

  /**
   * Stops the node and attempts to perform cleanup.
   *
   * @param input state unused
   * @return CallbackReturn success
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_DEBUG(this->get_logger(), "on_error");
    StopPublisher();
    if (publisher_ != nullptr) {
      publisher_.reset();
    }

    return CallbackReturn::SUCCESS;
  }

protected:
  /**
   * LifecyclePublisher publisher that periodically publishes statistic messages
   *
   */
  rclcpp_lifecycle::LifecyclePublisher<statistics_msgs::msg::MetricsMessage>::SharedPtr
    publisher_;

  /**
   * ROS2 topic statistics collectors
   */
  std::vector<std::unique_ptr<TopicStatisticsCollector<T>>> statistics_collectors_;

private:
  /**
   * Initialize topic statistics collectors to use with this node.
   */
  void InitializeCollectors()
  {
    statistics_collectors_.push_back(std::make_unique<ReceivedMessageAgeCollector<T>>());
    statistics_collectors_.push_back(std::make_unique<ReceivedMessagePeriodCollector<T>>());
  }

  /**
   * Creates ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   */
  void StartPublisher()
  {
    if (publisher_ == nullptr) {
      publisher_ = create_publisher<statistics_msgs::msg::MetricsMessage>(
        publish_topic_name_,
        10);
    }

    publisher_->on_activate();

    RCLCPP_DEBUG(this->get_logger(), "SetupStart: creating publish_timer_");
    publish_timer_ = this->create_wall_timer(
      publish_period_, [this]() {
        this->PublishStatisticMessage();
        this->ClearCollectorMeasurements();
      });
    window_start_ = now();
  }

  /**
   * Stops the ROS2 timers that were created by StartPublisher()
   *
   * @throws rcpputils::IllegalStateException if publish_timer_ or publisher_ are null
   */
  void StopPublisher()
  {
    rcpputils::check_true(publish_timer_ != nullptr);
    rcpputils::check_true(publisher_ != nullptr);

    ClearCollectorMeasurements();
    publisher_->on_deactivate();

    publish_timer_->cancel();
    publish_timer_.reset();
  }

  /**
  * Clear the emasurements made by all statistics collectors
  */
  void ClearCollectorMeasurements()
  {
    for (const auto & collector : statistics_collectors_) {
      collector->ClearCurrentMeasurements();
    }
  }

  /**
   * Publishes the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   *
   * @throws rcpputils::IllegalStateException if publisher_ is null or not activated
   */
  void PublishStatisticMessage() override
  {
    rcpputils::check_true(publisher_ != nullptr);
    rcpputils::check_true(publisher_->is_activated());

    this->window_start_ = this->now();

    for (const auto & collector : statistics_collectors_) {
      const auto msg = libstatistics_collector::collector::GenerateStatisticMessage(
        get_name(),
        collector->GetMetricName(),
        collector->GetMetricUnit(),
        window_start_,
        now(),
        collector->GetStatisticsResults());
      publisher_->publish(msg);
    }
  }

  /**
   * Tracks the starting time of the statistics
   */
  rclcpp::Time window_start_;

  /**
   * The period used to publish measurement data (defaults to 1000 ms)
   */
  std::chrono::milliseconds publish_period_{1000};

  /**
   * ROS2 timer used to publish measurement messages
   */
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * Subscriber to listen to incoming messages on a topic
   */
  std::vector<typename rclcpp::Subscription<T>::SharedPtr> subscriptions_;

  /**
   * Topic name to compute statistics for
   */
  std::vector<std::string> collect_topic_names_;

  /**
   * Topic name to publish collected statistics to
   */
  std::string publish_topic_name_;

public:
  /**
   * Returns multiple topic names
   */
  std::vector<std::string> GetCollectTopicName()
  {
    return collect_topic_names_;
  }
};
}  // namespace topic_statistics_collector
#endif  // TOPIC_STATISTICS_COLLECTOR__SUBSCRIBER_TOPIC_STATISTICS_HPP_
