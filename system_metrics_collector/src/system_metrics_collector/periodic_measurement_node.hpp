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

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "collector.hpp"
#include "metrics_message_publisher.hpp"

namespace system_metrics_collector
{

/**
 * Class which makes periodic measurements, using a ROS2 timer.
 */
class PeriodicMeasurementNode : public system_metrics_collector::Collector,
  public system_metrics_collector::MetricsMessagePublisher, public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * Construct a PeriodicMeasurementNode.
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
   * Return a pretty printed status representation of this class
   *
   * @return a string detailing the current status
   */
  std::string GetStatusString() const override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state);

protected:
  /**
   * Create ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   *
   * @return if setup was successful
   */
  bool SetupStart() override;

  /**
   * Stop the ROS2 timers that were created by SetupStart()
   *
   * @return if teardown was successful
   */
  bool SetupStop() override;

  /**
   * Track the starting time of the statistics
   */
  rclcpp::Time window_start_;

  rclcpp_lifecycle::LifecyclePublisher<metrics_statistics_msgs::msg::MetricsMessage>::SharedPtr
    publisher_;

private:
  /**
   * Override this method to perform a single measurement. This is called via PerformPeriodicMeasurement
   * with the period defined in the constructor.
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
   * Publish the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   */
  void PublishStatisticMessage() override;

  /**
   * The period used to take a single measurement
   */
  std::chrono::milliseconds measurement_period_;
  /**
   * The period used to publish measurement data
   */
  std::chrono::milliseconds publish_period_;

  rclcpp::TimerBase::SharedPtr measurement_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_
