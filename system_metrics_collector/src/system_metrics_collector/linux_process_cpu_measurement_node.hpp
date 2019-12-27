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
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_pid_cpu_data.hpp"

#include "rclcpp/rclcpp.hpp"


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
   *
   * @param name the name of this node
   * @param measurement_period the period of this node, used to read measurements
   * @param topic the topic name used for publishing
   * @param publish_period the period at which metrics are published.
   */
  LinuxProcessCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period);

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
   * This obtains measurements from clock_gettime() to obtain process-specific and system-wide CPU
   * used.
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

private:
  /**
   * The pid of this process.
   */
  const std::string pid_;
  /**
   * The cached processa and system measurements used in order to perform the CPU active percentage.
   */
  ProcPidCpuData last_measurement_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_CPU_MEASUREMENT_NODE_HPP_
