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

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"

namespace
{
constexpr const char PROC_STAT_FILE[] = "/proc/meminfo";
constexpr const char MEM_TOTAL[] = "MemTotal:";
constexpr const char MEM_AVAILABLE[] = "MemAvailable:";
}  // namespace

LinuxMemoryMeasurementNode::LinuxMemoryMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds & publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period)
{
}

double LinuxMemoryMeasurementNode::periodicMeasurement()
{
  double percentage_used = std::nan("");

  std::ifstream stat_file(PROC_STAT_FILE);
  std::string line;

  std::getline(stat_file, line);  // MemTotal
  if (!stat_file.good()) {
    return percentage_used;
  }

  std::istringstream ss(line);
  if (!ss.good()) {
    return percentage_used;
  }
  std::string label;
  ss >> label;
  if (!ss.good() || (label != MEM_TOTAL)) {
    return std::nan("");
  }

  double total = 0.0;
  ss >> total;
  if (!ss.good()) {
    return percentage_used;
  }

  std::getline(stat_file, line);  // skip MemFree

  std::getline(stat_file, line);  // MemAvailable
  if (!stat_file.good()) {
    return percentage_used;
  }

  ss.clear();
  ss.str(line);
  label.clear();
  ss >> label;

  if (!ss.good() || (label != MEM_AVAILABLE)) {
    return std::nan("");
  }

  if (!ss.good()) {
    return percentage_used;
  }
  double available = 0.0;
  ss >> available;

  percentage_used = (total - available) / (total) * 100.0;
  return percentage_used;
}
