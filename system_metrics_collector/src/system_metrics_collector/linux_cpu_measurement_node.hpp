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

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/cpu_data.hpp"

/**
 * Node that periodically calculates the % of active CPU by
 * reading /proc/stat.
 */
class LinuxCpuMeasurementNode : public PeriodicMeasurementNode
{
public:
  static constexpr const char PROC_STAT_FILE[] = "/proc/stat";
  static constexpr const char CPU_LABEL[] = "cpu";
  static constexpr const size_t CPU_LABEL_LENGTH = 3;
  /**
   *
   * @param name
   * @param measurement_period
   * @param topic
   */
  LinuxCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds & publish_period =
    PeriodicMeasurementNode::DEFAULT_PUBLISH_WINDOW);

  virtual ~LinuxCpuMeasurementNode() = default;

  /**
   * Parse a line read from /proc/stat
   *
   * @param stat_cpu_line a line from /proc/stat
   * @return ProcCpuData struct populated if parsed, otherwise empty
   */
  static ProcCpuData processLine(const std::string & stat_cpu_line);

  /**
   * Compute the percentage of CPU active.
   *
   * @param measurement1 the first measurement
   * @param measurement2 the second measurement (made after the first)
   * @return percentage of CPU active
   */
  static double computeCpuActivePercentage(
    const ProcCpuData & measurement1,
    const ProcCpuData & measurement2);

protected:
  /**
   * Perform a periodic measurement calculating the percentage of CPU active.
   *
   * @return percentage of CPU active
   */
  double periodicMeasurement() override;

private:
  /**
   * Perform a single measurement of cpu data by reading /proc/stat.
   *
   * @return ProcCpuData the measurement made
   */
  virtual ProcCpuData makeSingleMeasurement();
};

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_CPU_MEASUREMENT_NODE_HPP_