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

#ifndef SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_CPU_MEASUREMENT_NODE_HPP_
#define SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_CPU_MEASUREMENT_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>
#include <tuple>

#include "periodic_measurement_node.hpp"
#include "proc_cpu_data.hpp"
#include "utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


namespace system_metrics_collector
{

/**
 * Measures the CPU percentage used by the process.
 */
class LinuxProcessCpuMeasurementNode : public PeriodicMeasurementNode
{
public:
  /**
   * Constructs a LinuxProcessCpuMeasurementNode.
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node
   * @param options the options (arguments, parameters, etc.) for this node
   */
  LinuxProcessCpuMeasurementNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /**
   * Constructs a LinuxProcessCpuMeasurementNode and starts it.
   * The node name will be "linux_process_cpu_collector" by default
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param options the options (arguments, parameters, etc.) for this node
   */
  explicit LinuxProcessCpuMeasurementNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

protected:
  /**
   * Creates ROS2 timers and a publisher for periodically triggering measurements and publishing
   * MetricsMessages.
   * This function needs to be overridden because this class needs to also reset last_measurement_
   * on every start or restart in addition to what is done by PeriodicMeasurementNode.
   *
   * @return if setup was successful
   */
  bool SetupStart() override;

  /**
   * Performs a periodic measurement and calculation of the percentage of CPU this process used.
   * This obtains measurements from MakeSingleMeasurement() to obtain process-specific
   * and system-wide CPU used.
   *
   * @return percentage of CPU this process used
   */
  double PeriodicMeasurement() override;

  /**
   * Returns the name to use for this metric.
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

private:
  /**
   * Performs a single measurement of CPU data by using clock_gettime().
   *
   * @return ProcCpuData the measurement made
   */
  virtual system_metrics_collector::ProcPidCpuData MakeSingleMeasurement();

  /**
   * The pid of this process.
   */
  const std::string metric_name_;
  /**
   * The cached process and system measurements used in order to perform the CPU active percentage.
   */
  ProcPidCpuData last_measurement_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_CPU_MEASUREMENT_NODE_HPP_
