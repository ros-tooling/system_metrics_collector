// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


#ifndef SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_
#define SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_

#include <chrono>
#include <string>

#include "statistics_msgs/msg/metrics_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "libstatistics_collector/collector/collector.hpp"
#include "metrics_message_publisher.hpp"

namespace system_metrics_collector
{

/**
 * Class which makes periodic measurements, using a ROS2 timer.
 */
class PeriodicMeasurementNode : public libstatistics_collector::collector::Collector,
  public system_metrics_collector::MetricsMessagePublisher, public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Constructs a PeriodicMeasurementNode.
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node. This must be non-empty.
   * @param options the options (arguments, parameters, etc.) for this node
   * @throws std::invalid_argument for any invalid input
   */
  PeriodicMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options);

  virtual ~PeriodicMeasurementNode() = default;

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

protected:
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
   * Tracks the starting time of the statistics
   */
  rclcpp::Time window_start_;

  /**
   * LifecyclePublisher publisher that is activated on SetupStart and deactivated on SetupStop().
   */
  rclcpp_lifecycle::LifecyclePublisher<statistics_msgs::msg::MetricsMessage>::SharedPtr
    publisher_;

private:
  /**
   * Override this method to perform a single measurement. This is called via
   * PerformPeriodicMeasurement with the period defined in the constructor.
   *
   * @return the measurement made to be aggregated for statistics
   */
  virtual double PeriodicMeasurement() = 0;

  /**
   * Called via a ROS2 timer per the measurement_period_. This calls PeriodicMeasurement
   * and adds the resulting output via Collector::AcceptData(double data);
   */
  virtual void PerformPeriodicMeasurement();

  /**
   * Publishes the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   */
  void PublishStatisticMessage() override;

  /**
   * The period used to take a single measurement
   */
  std::chrono::milliseconds measurement_period_{0};
  /**
   * The period used to publish measurement data
   */
  std::chrono::milliseconds publish_period_{0};

  /**
   * ROS2 timer used to trigger collection measurements.
   */
  rclcpp::TimerBase::SharedPtr measurement_timer_;

  /**
   * ROS2 timer used to publish measurement messages.
   */
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_
