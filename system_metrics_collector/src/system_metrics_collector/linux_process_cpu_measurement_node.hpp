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

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


namespace system_metrics_collector
{

/**
 * Class used to measure the memory percentage used as a process.
 */
class LinuxProcessCpuMeasurementNode : public PeriodicMeasurementNode
{
public:
  /**
   * Construct a LinuxProcessCpuMeasurementNode
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
   * Create ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   * This function needs to be overridden because this class needs to also reset
   * last_process_measurement_ and last_system_measurement_ on every start or restart
   * in addition to what is done by PeriodicMeasurementNode
   *
   * @return if setup was successful
   */
  bool setupStart() override;

  /**
   * Perform a periodic measurement calculating the percentage of
   * memory this process used. This reads and parses the /proc/<pid>/statm file
   * to obtain the process memory used and divides that by the total system
   * memory, which is obtained through sys/sysinfo.h.
   *
   * @return percentage of memory this process used
   */
  double periodicMeasurement() override;

  /**
   * Return the name to use for this metric
   * @return a string of the name for this measured metric
   */
  std::string getMetricName() const override;

private:
  /**
   * Perform a single measurement of cpu data by reading /proc/<pid>/stat.
   *
   * @return ProcCpuData the measurement made
   */
  virtual std::tuple<ProcPidCpuData, ProcCpuData> makeSingleMeasurement();

  /**
   * The pid of this process/
   */
  const std::string pid_;
  /**
   * The stat file to read for this process.
   */
  const std::string file_to_read_;
  /**
   * The cached process-specific measurement used in order to perform the CPU active percentage.
   */
  ProcPidCpuData last_process_measurement_;
  /**
   * The cached system measurement used in order to perform the CPU active percentage.
   */
  ProcCpuData last_system_measurement_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_CPU_MEASUREMENT_NODE_HPP_
