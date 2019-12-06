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

#include <sys/sysinfo.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <sstream>
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
constexpr const char PROC[] = "/proc/";
constexpr const char STATM[] = "/statm";
constexpr const char METRIC_NAME[] = "memory_percent_used";
}  // namespace

namespace system_metrics_collector
{

class LinuxProcessMemoryMeasurementNode : public PeriodicMeasurementNode
{
public:
  LinuxProcessMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period)
  : PeriodicMeasurementNode(name, measurement_period, topic, publish_period),
    pid_(std::to_string(getPid())),
    file_to_read_(PROC + pid_ + STATM)
  {
  }

protected:
  double periodicMeasurement() override
  {
    // get the total amount of memory
    struct sysinfo si;
    sysinfo(&si);

    // read the /proc/<pid>/statm file to get total process memory
    const auto data = readFileToString(file_to_read_);
    std::istringstream ss(data);
    if (ss.good()) {
      int process_memory_used;
      ss >> process_memory_used;
      if (ss.good()) {
        return static_cast<double>(process_memory_used) /
               static_cast<double>(si.totalram) * 100.0;
      }
    }

    RCLCPP_ERROR(this->get_logger(), "unable to make measurement");
    return std::nan("");
  }

  std::string getMetricName() const override
  {
    return pid_ + METRIC_NAME;
  }

private:
  const std::string pid_;
  const std::string file_to_read_;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP_
