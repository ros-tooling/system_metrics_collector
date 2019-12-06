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

#ifndef SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP
#define SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP

#include <sys/sysinfo.h>
#include <sys/types.h>

#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
    constexpr const char PROC[] = "/proc/";
    constexpr const char STATM[] = "/statm";
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
    file_to_read_(PROC + std::to_string(getPid()) + STATM)
  {
  }

protected:
  double periodicMeasurement() override
  {
    // get the total amount of memory
    struct sysinfo si;
    sysinfo(&si);

    const auto data = readFileToString(file_to_read_);
    std::istringstream ss(data);
    if (ss.good()) {
      int process_memory_used;
      ss >> process_memory_used;
      if (ss.good()) {
        std::cout << (double) process_memory_used / (double) si.totalram * 100.0 << std::endl;
        return (double) process_memory_used / (double) si.totalram * 100.0;
      }
    }

    return std::nan("");
  }

void publishStatisticMessage()
{
  auto msg = generateStatisticMessage(get_name(), "asdf", window_start_,
                                      now(), getStatisticsResults());
  publisher_->publish(msg);
}


private:
  const std::string file_to_read_;
};

}  // namespace system_metrics_collector

#endif //SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP
