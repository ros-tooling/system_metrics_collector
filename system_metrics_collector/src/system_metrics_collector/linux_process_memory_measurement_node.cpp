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

#include "linux_process_memory_measurement_node.hpp"

#include <sys/sysinfo.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/logging_macros.h"

#include "constants.hpp"

namespace
{

constexpr const char kProc[] = "/proc/";
constexpr const char kStatm[] = "/statm";
constexpr const char kMetricName[] = "_memory_percent_used";

/**
 * Return the total system memory.
 *
 * @return the total system memory in bytes
 */
double GetSystemTotalMemory()
{
  // the following needs the struct keyword because sysinfo is also a function name
  struct sysinfo si {};
  const auto success = sysinfo(&si);
  return success == -1 ? std::nan("") : static_cast<double>(si.totalram);
}

}  // namespace

namespace system_metrics_collector
{

LinuxProcessMemoryMeasurementNode::LinuxProcessMemoryMeasurementNode(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: PeriodicMeasurementNode{name, options},
  pid_{std::to_string(GetPid())},
  file_to_read_{kProc + std::to_string(GetPid()) + kStatm}
{
}

LinuxProcessMemoryMeasurementNode::LinuxProcessMemoryMeasurementNode(
  const rclcpp::NodeOptions & options)
: LinuxProcessMemoryMeasurementNode{"linux_process_memory_collector", options}
{
  configure();
  activate();
}

double LinuxProcessMemoryMeasurementNode::PeriodicMeasurement()
{
  const auto statm_line = ReadFileToString(file_to_read_);
  double p_mem;
  try {
    p_mem = static_cast<double>(GetProcessUsedMemory(statm_line));
  } catch (const std::ifstream::failure & e) {
    RCLCPP_ERROR(
      this->get_logger(), "caught %s, failed to GetProcessUsedMemory from line %s",
      e.what(), file_to_read_.c_str());
    return std::nan("");
  }
  const auto total_mem = GetSystemTotalMemory();

  return p_mem / total_mem * 100.0;
}

std::string LinuxProcessMemoryMeasurementNode::GetMetricName() const
{
  return pid_ + kMetricName;
}

std::string LinuxProcessMemoryMeasurementNode::GetMetricUnit() const
{
  return collector_node_constants::kPercentUnitName;
}

uint64_t GetProcessUsedMemory(const std::string & statm_process_file_contents)
{
  std::istringstream ss{statm_process_file_contents};
  ss.exceptions(std::ios::failbit | std::ios::badbit);
  uint64_t process_memory_used;
  ss >> process_memory_used;
  return process_memory_used;
}

}   // namespace system_metrics_collector


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(system_metrics_collector::LinuxProcessMemoryMeasurementNode);
