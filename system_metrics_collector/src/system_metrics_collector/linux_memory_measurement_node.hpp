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

#include "periodic_measurement_node.hpp"

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
   * The following parameters may be set via the rclcpp::NodeOptions:
   * `measurement_period`: the period of this node, used to read measurements
   * `publish_period`: the period at which metrics are published
   *
   * @param name the name of this node
   * @param options the options (arguments, parameters, etc.) for this node
   */
  LinuxMemoryMeasurementNode(
    const std::string & name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

  virtual ~LinuxMemoryMeasurementNode() = default;

protected:
  /**
   * Perform a periodic measurement calculating the percentage of
   * RAM used. This reads and parses the /proc/meminfo file
   * to calculate memory used.
   *
   * @return percentage of RAM used
   */
  double PeriodicMeasurement() override;

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
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_MEMORY_MEASUREMENT_NODE_HPP_
