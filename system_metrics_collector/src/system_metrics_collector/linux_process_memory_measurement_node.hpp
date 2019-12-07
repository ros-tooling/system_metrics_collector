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

#ifndef SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP_
#define SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP_

#include <chrono>
#include <cmath>
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
constexpr const char PROC[] = "/proc/";
constexpr const char STATM[] = "/statm";
constexpr const char METRIC_NAME[] = "_memory_percent_used";
}  // namespace

namespace system_metrics_collector
{

/**
* Return the number of bytes used after parsing a process's statm file.
*
* @param statm_process_file the statm file to parse
* @return the number of bytes used for the statm file's process
*/
double getProcessUsedMemory(const std::string & statm_process_file_contents);

/**
 * Return the total system memory.
 *
 * @return the total system memory in bytes
 */
double getSystemTotalMemory();

/**
 * Class used to measure the memory percentage used as a process.
 */
class LinuxProcessMemoryMeasurementNode : public PeriodicMeasurementNode
{
public:
  /**
   * Construct a LinuxProcessMemoryMeasurementNode
   *
   * @param name the name of this node
   * @param measurement_period the period of this node, used to read measurements
   * @param topic the topic name used for publishing
   * @param publish_period the period at which metrics are published.
   */
  LinuxProcessMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period);

private:
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

  /**
   * The pid of this process/
   */
  const std::string pid_;
  /**
   * The statm file to read for this process.
   */
  const std::string file_to_read_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP_
