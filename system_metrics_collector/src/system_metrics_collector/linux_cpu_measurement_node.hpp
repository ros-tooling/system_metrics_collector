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
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"

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
   * Construct a LinuxCpuMeasurementNode
   *
   * @param name the name of this node
   * @param measurement_period the period of this node, used to read measurements
   * @param topic the topic name used for publishing
   * @param publish_period the period at which metrics are published. 0 ms means don't publish
   */
  LinuxCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period);

  virtual ~LinuxCpuMeasurementNode() = default;

protected:
  /**
   * Create ROS2 timers and a publisher for periodically triggering measurements
   * and publishing MetricsMessages
   *
   * @return if setup was successful
   */
  bool setupStart() override;

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
  virtual system_metrics_collector::ProcCpuData makeSingleMeasurement();

  /**
   * Return the name to use for this metric
   * @return a string of the name for this measured metric
   */
  std::string getMetricName() const override;

  /**
   * The cached measurement used in order to perform the CPU active
   * percentage.
   */
  system_metrics_collector::ProcCpuData last_measurement_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_CPU_MEASUREMENT_NODE_HPP_
