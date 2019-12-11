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

#include "../../src/system_metrics_collector/linux_process_memory_measurement_node.hpp"

#include <sys/sysinfo.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "rcutils/logging_macros.h"

namespace
{

constexpr const char PROC[] = "/proc/";
constexpr const char STATM[] = "/statm";
constexpr const char METRIC_NAME[] = "_memory_percent_used";

/**
 * Return the total system memory.
 *
 * @return the total system memory in bytes
 */
double getSystemTotalMemory()
{
  struct sysinfo si;
  const auto success = sysinfo(&si);
  return success == -1 ? std::nan("") : static_cast<double>(si.totalram);
}

}  // namespace

namespace system_metrics_collector
{

LinuxProcessMemoryMeasurementNode::LinuxProcessMemoryMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period),
  pid_(std::to_string(getPid())),
  file_to_read_(PROC + pid_ + STATM)
{
}

double LinuxProcessMemoryMeasurementNode::periodicMeasurement()
{
  const auto statm_line = readFileToString(file_to_read_);
  double p_mem;
  try {
    p_mem = static_cast<double>(getProcessUsedMemory(statm_line));
  } catch (std::ifstream::failure e) {
    RCLCPP_ERROR(
      this->get_logger(), "caught %s, failed to getProcessUsedMemory from line %s",
      e.what(), file_to_read_);
    return std::nan("");
  }
  const auto total_mem = getSystemTotalMemory();

  return p_mem / total_mem * 100.0;
}

std::string LinuxProcessMemoryMeasurementNode::getMetricName() const
{
  return pid_ + METRIC_NAME;
}

uint64_t getProcessUsedMemory(const std::string & statm_process_file_contents)
{
  std::istringstream ss{statm_process_file_contents};
  ss.exceptions(std::ios::failbit | std::ios::badbit);
  uint64_t process_memory_used;
  ss >> process_memory_used;
  return process_memory_used;
}

}   // namespace system_metrics_collector
