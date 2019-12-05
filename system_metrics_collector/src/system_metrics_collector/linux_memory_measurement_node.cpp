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

#include <chrono>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <string>

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
constexpr const char MEASUREMENT_TYPE[] = "system_memory_usage";
constexpr const char PROC_STAT_FILE[] = "/proc/meminfo";
}  // namespace

namespace system_metrics_collector
{

LinuxMemoryMeasurementNode::LinuxMemoryMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period)
{
}

double LinuxMemoryMeasurementNode::periodicMeasurement()
{
  std::ifstream stat_file(PROC_STAT_FILE);
  if (!stat_file.good()) {
    return std::nan("");
  }
  auto read_string = readFileToString(PROC_STAT_FILE);
  return processMemInfoLines(read_string);
}

void LinuxMemoryMeasurementNode::publishStatisticMessage()
{
  auto msg = generateStatisticMessage(get_name(), MEASUREMENT_TYPE, window_start_,
      now(), getStatisticsResults());
  publisher_->publish(msg);
}

}  // namespace system_metrics_collector
