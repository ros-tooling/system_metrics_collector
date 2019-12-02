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

#include "collector.hpp"

#include "rclcpp/rclcpp.hpp"

namespace system_metrics_collector
{

/**
 * Class which makes periodic measurements, using a ROS2 timer.
 */
class PeriodicMeasurementNode : public system_metrics_collector::Collector, public rclcpp::Node
{
public:
  static constexpr const std::chrono::milliseconds INVALID_PUBLISH_WINDOW =
    std::chrono::milliseconds(0);
  /**
   * Construct a PeriodicMeasurementNode.
   *
   * @param name the name of this node
   * @param topic the topic for publishing data
   * @param measurement_period
   * @param publish_period the window of active measurements. If specified all measurements
   * will be cleared when the window has been exceeded.
   */
  PeriodicMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,  // todo @dbbonnie think about a default topic
    const std::chrono::milliseconds publish_period = INVALID_PUBLISH_WINDOW,
    const bool clear_measurements_on_publish = true);

  virtual ~PeriodicMeasurementNode() = default;

  /**
   * Return a pretty printed status representation of this class
   *
   * @return a string detailing the current status
   */
  std::string getStatusString() const override;

private:
  /**
   * Override this method to perform a single measurement. This is called via performPeriodicMeasurement
   * with the period defined in the constructor.
   *
   * @return the measurement made to be aggregated for statistics
   */
  virtual double periodicMeasurement() = 0;

  /**
   * Called via a ROS2 timer per the measurement_period_. This calls periodicMeasurement
   * and adds the resulting output via Collector::acceptData(double data);
   */
  virtual void performPeriodicMeasurement();

  /**
   * Creates a ROS2 timer with a period of measurement_period_.
   *
   * @return if setup was successful
   */
  bool setupStart() override;

  /**
   * Stops the ROS2 timer
   *
   * @return if teardown was successful
   */
  bool setupStop() override;

  // todo implement on publish timer callback, check if we need to clear the window
  // todo this is part of the publishing interface

  /**
   * Topic used for publishing
   */
  const std::string publishing_topic_;
  /**
   * The period used to take a single measurement
   */
  const std::chrono::milliseconds measurement_period_;
  /**
   * The period used to publish measurement data
   */
  const std::chrono::milliseconds publish_period_;
  /**
   * If true, then measurements are cleared after publishing data
   */
  const bool clear_measurements_on_publish_;
  rclcpp::TimerBase::SharedPtr measurement_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_
