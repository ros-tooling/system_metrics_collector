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

#ifndef SYSTEM_METRICS_COLLECTOR__LINUX_CPU_MEASUREMENT_NODE_HPP_
#define SYSTEM_METRICS_COLLECTOR__LINUX_CPU_MEASUREMENT_NODE_HPP_

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "periodic_measurement_node.hpp"
#include "proc_cpu_data.hpp"

namespace system_metrics_collector
{

/**
 * Node that periodically calculates the % of active CPU by
 * reading /proc/stat.
 */
class LinuxCpuMeasurementNode : public system_metrics_collector::PeriodicMeasurementNode
{
public:
  /**
   * Construct a LinuxCpuMeasurementNode.
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node
   * @param options the options (arguments, parameters, etc.) for this node
   */
  LinuxCpuMeasurementNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  virtual ~LinuxCpuMeasurementNode() = default;

protected:
  /**
   * Create ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   *
   * @return if setup was successful
   */
  bool SetupStart() override;

  /**
   * Perform a periodic measurement calculating the percentage of CPU active.
   *
   * @return percentage of CPU active
   */
  double PeriodicMeasurement() override;

private:
  /**
   * Perform a single measurement of cpu data by reading /proc/stat.
   *
   * @return ProcCpuData the measurement made
   */
  virtual system_metrics_collector::ProcCpuData MakeSingleMeasurement();

  /**
   * Returns the name to use for this metric
   *
   * @return a string of the name for this measured metric
   */
  std::string GetMetricName() const override;

  /**
   * Returns the name of the measurement unit of this metric
   *
   * @return a string of the name of the measurement unit of this metric
   */
  std::string GetMetricUnit() const override;

  /**
   * The cached measurement used in order to perform the CPU active
   * percentage.
   */
  system_metrics_collector::ProcCpuData last_measurement_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_CPU_MEASUREMENT_NODE_HPP_
