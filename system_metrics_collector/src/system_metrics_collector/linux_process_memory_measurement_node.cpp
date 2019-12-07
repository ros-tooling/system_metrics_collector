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
#include <sstream>
#include <string>

#include "rcutils/logging_macros.h"

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
  const auto p_mem = getProcessUsedMemory(statm_line);
  const auto total_mem = getSystemTotalMemory();

  return p_mem / total_mem * 100.0;
}

std::string LinuxProcessMemoryMeasurementNode::getMetricName() const
{
  return pid_ + METRIC_NAME;
}

double getProcessUsedMemory(
  const std::string & statm_process_file_contents)
{
  std::istringstream ss(statm_process_file_contents);
  if (ss.good()) {
    size_t process_memory_used;
    ss >> process_memory_used;
    if (ss.good()) {
      return static_cast<double>(process_memory_used);
    }
  }

  RCUTILS_LOG_ERROR_NAMED("getProcessUsedMemory",
    "unable to parse contents: %s", statm_process_file_contents.c_str());
  return std::nan("");
}

double getSystemTotalMemory()
{
  struct sysinfo si;
  auto success = sysinfo(&si);
  return success == -1 ? std::nan("") : static_cast<double>(si.totalram);
}

}   // namespace system_metrics_collector
