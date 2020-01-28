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

#include "periodic_measurement_node.hpp"
#include "utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


namespace system_metrics_collector
{

/**
* Return the number of bytes used after parsing a process's statm file.
*
* @param statm_process_file the statm file to parse
* @return the number of bytes used for the statm file's process
 *@throws std::ifstream::failure for std::ios::failbit | std::ios::badbit
*/
uint64_t GetProcessUsedMemory(const std::string & statm_process_file_contents);

/**
 * Class used to measure the memory percentage used as a process.
 */
class LinuxProcessMemoryMeasurementNode : public PeriodicMeasurementNode
{
public:
  /**
   * Constructs a LinuxProcessMemoryMeasurementNode.
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node
   * @param options the options (arguments, parameters, etc.) for this node
   */
  LinuxProcessMemoryMeasurementNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  /**
   * Constructs a LinuxProcessMemoryMeasurementNode and starts it.
   * The node name will be "linux_process_memory_collector" by default
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node
   * @param options the options (arguments, parameters, etc.) for this node
   */
  explicit LinuxProcessMemoryMeasurementNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

protected:
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
  const std::string & GetMetricUnit() const override;

private:
  /**
   * Perform a periodic measurement calculating the percentage of
   * memory this process used. This reads and parses the /proc/<pid>/statm file
   * to obtain the process memory used and divides that by the total system
   * memory, which is obtained through sys/sysinfo.h.
   *
   * @return percentage of memory this process used
   */
  double PeriodicMeasurement() override;

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
