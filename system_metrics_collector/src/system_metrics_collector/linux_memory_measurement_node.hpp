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

/**
 * Read the contents of the input filename into a string. Helper function
 * for parsing.
 *
 * @param file_name the file to be read
 * @return the file to be read's contents as a std::string
 */
std::string readFile(const std::string & file_name);

/**
 * Process input lines from the /proc/meminfo file. The expected format to
 * compute memory used is
 *
 *    MemTotal:       16302048 kB
 *    MemFree:          200660 kB
 *    MemAvailable:    9523668 kB
 *
 * @param lines the lines from the /proc/meminfo file
 * @return the percentage of memory used, specifically
 * (MemTotal - MemAvailable) / MemTotal * 100.0
 */
double processLines(const std::string & lines);

/**
 * Node that periodically measures the percentage of RAM used
 * by a linux system. Specifically, the values used to make
 * this measurement are obtained from /proc/meminfo.
 */
class LinuxMemoryMeasurementNode : public PeriodicMeasurementNode
{
public:
  /**
   * Construct a LinuxMemoryMeasurementNode
   *
   * @param name the name of this node
   * @param measurement_period the period of this node, used to
   * read measurements
   * @param topic the topic name used for publishing
   */
  LinuxMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period =
    PeriodicMeasurementNode::DEFAULT_PUBLISH_WINDOW);

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

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_MEMORY_MEASUREMENT_NODE_HPP_
