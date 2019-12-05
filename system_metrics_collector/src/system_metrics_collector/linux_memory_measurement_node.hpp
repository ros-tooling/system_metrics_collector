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

#ifndef SYSTEM_METRICS_COLLECTOR__LINUX_MEMORY_MEASUREMENT_NODE_HPP_
#define SYSTEM_METRICS_COLLECTOR__LINUX_MEMORY_MEASUREMENT_NODE_HPP_

#include <chrono>
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"

namespace system_metrics_collector
{
/**
 * Node that periodically measures the percentage of RAM used
 * by a linux system. Specifically, the values used to make
 * this measurement are obtained from /proc/meminfo.
 */
class LinuxMemoryMeasurementNode : public system_metrics_collector::PeriodicMeasurementNode
{
public:
  /**
   * Construct a LinuxMemoryMeasurementNode
   *
   * @param name the name of this node
   * @param measurement_period the period of this node, used to read measurements
   * @param topic the topic name used for publishing
   * @param publish_period the period at which metrics are published. 0 ms means don't publish
   */
  LinuxMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period);

  virtual ~LinuxMemoryMeasurementNode() = default;

protected:
  /**
   * Perform a periodic measurement calculating the percentage of
   * RAM used. This reads and parses the /proc/meminfo file
   * to calculate memory used.
   *
   * @return percentage of RAM used
   */
  double periodicMeasurement() override;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_MEMORY_MEASUREMENT_NODE_HPP_
