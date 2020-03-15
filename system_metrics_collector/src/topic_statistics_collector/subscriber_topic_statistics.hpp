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
#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include "system_metrics_collector/metrics_message_publisher.hpp"
#include "topic_statistics_collector.hpp"

namespace topic_statistics_collector
{

/**
 * Class which makes periodic topic statistics measurements, using a ROS2 timer.
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
   *
   * @param node_name the name of this node, it must be non-empty
   * @param node_options the options (arguments, parameters, etc.) for this node
   * @param topic_name the name of topic to compute statistics for
   */
  SubscriberTopicStatisticsNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options);

  virtual ~SubscriberTopicStatisticsNode();

  /**
   * Starts the node.
   *
   * @param state input state unused
   * @return CallbackReturn success if start returns true, error otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);

  /**
   * Stops the node.
   *
   * @param input state unused
   * @return CallbackReturn success if start returns true, error otherwise
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);

  /**
   * Stops the node and performs cleanup.
   *
   * @param input state unused
   * @return CallbackReturn success
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state);

  /**
   * Stops the node and attempts to perform cleanup.
   *
   * @param input state unused
   * @return CallbackReturn success
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state);

protected:
  /**
   * LifecyclePublisher publisher that periodically publishes statistic messages
   *
   */
  rclcpp_lifecycle::LifecyclePublisher<metrics_statistics_msgs::msg::MetricsMessage>::SharedPtr
    publisher_;

  /**
  * ROS 2 topic statistics collectors
  */
  std::vector<std::shared_ptr<TopicStatisticsCollector<T>>> statistics_collectors_;

private:
  /**
   * Creates ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   *
   * @return if setup was successful
   */
  void StartPublisher();

  /**
   * Stops the ROS2 timers that were created by StartPublisher()
   *
   * @return if teardown was successful
   */
  void StopPublisher();

  /**
  * Clear the emasurements made by all statistics collectors
  */
  void ClearCollectorMeasurements();

  /**
   * Publishes the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   */
  void PublishStatisticMessage() override;

  /**
   * Clock to use to get current time needed for statistic calculation
   */
  rcl_clock_t clock_;

  /**
   * Tracks the starting time of the statistics
   */
  rclcpp::Time window_start_;

  /**
   * The period used to publish measurement data
   */
  std::chrono::milliseconds publish_period_{0};

  /**
   * ROS2 timer used to publish measurement messages
   */
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /**
   * Subscriber to listen to incoming messages on a topic
   */
  typename rclcpp::Subscription<T>::SharedPtr subscription_;

  /**
   * Topic name to compute statistics for
   */
  std::string collect_topic_name_;

  /**
   * Topic name to compute statistics for
   */
  std::string publish_topic_name_;
};

}  // namespace topic_statistics_collector
#endif  // TOPIC_STATISTICS_COLLECTOR__SUBSCRIBER_TOPIC_STATISTICS_HPP_
