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

#include "received_message_age.hpp"
#include "received_message_period.hpp"
#include "system_metrics_collector/collector.hpp"
#include "system_metrics_collector/metrics_message_publisher.hpp"

namespace topic_statistics_collector
{

/**
 * Class which makes periodic topic statistics measurements, using a ROS2 timer.
 */
template<typename T>
class SubscriberTopicStatistics : public system_metrics_collector::MetricsMessagePublisher,
  public rclcpp_lifecycle::LifecycleNode,
  public system_metrics_collector::Collector
{
public:
  /**
   * Constructs a SubscriberTopicStatistics node.
   * The following parameter can be set via the rclcpp::NodeOptions:
   * `publish_period`: the period at which metrics are published
   *
   * @param node_name the name of this node, it must be non-empty
   * @param node_options the options (arguments, parameters, etc.) for this node
   * @param topic_name the name of topic to compute statistics for
   */
  SubscriberTopicStatistics(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options,
    const std::string & topic_name);

  virtual ~SubscriberTopicStatistics() = default;

  /**
   * Returns a pretty printed status representation of this class
   *
   * @return a string detailing the current status
   */
  std::string GetStatusString() const override;

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

  /**
   * Creates ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   *
   * @return if setup was successful
   */
  bool SetupStart() override;

  /**
   * Stops the ROS2 timers that were created by SetupStart()
   *
   * @return if teardown was successful
   */
  bool SetupStop() override;

  /**
   * LifecyclePublisher publisher that is activated on SetupStart and deactivated on SetupStop().
   */
  rclcpp_lifecycle::LifecyclePublisher<metrics_statistics_msgs::msg::MetricsMessage>::SharedPtr
    publisher_;

private:
  /**
   * Publishes the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   */
  void PublishStatisticMessage() override;

  /**
   *  Callback function to execute when messages on subscribed topics are received
   *
   *  @param msg message received
  **/
  void CollectorCallback(const typename T::SharedPtr received_message);

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
  * Subscription options to configure the subscriber
  */
  rclcpp::SubscriptionOptions subscription_options_;

  /**
  * Topic name to compute statistics for
  */
  const std::string topic_name_;

  /**
  * ROS2 message age calculator
  */
  std::vector<std::unique_ptr<TopicStatisticsCollector<T>>> statistics_collectors_;
};

}  // namespace topic_statistics_collector
#endif  // TOPIC_STATISTICS_COLLECTOR__SUBSCRIBER_TOPIC_STATISTICS_HPP_
