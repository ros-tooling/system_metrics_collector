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

/**
 * Class which makes periodic measurements, using a ROS2 timer.
 */
class PeriodicMeasurementNode : public Collector, public rclcpp::Node
{
public:
  /**
   * Construct a PeriodicMeasurementNode.
   *
   * @param name the name of this node
   * @param topic the topic for publishing data
   * @param measurement_period
   */
  explicit PeriodicMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds & measurement_period,
    const std::string & topic);  // todo @dbbonnie think about a default topic

  virtual ~PeriodicMeasurementNode() = default;

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

  /**
   * Periodically calls this method, via the ROS2 timer, in order to perform a
   * measurement. The measurement should be defined in the periodicMeasurement method.
   */
  void performPeriodicMeasurement();

  /**
   * Return a pretty printed status representation of this class
   *
   * @return a string detailing the current status
   */
  std::string getStatusString() override;

protected:
  /**
   * Override this method to perform a measurement. This is called via performPeriodicMeasurement
   * with the period defined in the constructor.
   */
  virtual void periodicMeasurement() = 0;

private:
  std::string publishing_topic_;
  std::chrono::milliseconds measurement_period_;
  rclcpp::TimerBase::SharedPtr measurement_timer_{nullptr};
};


#endif  // SYSTEM_METRICS_COLLECTOR__PERIODIC_MEASUREMENT_NODE_HPP_
