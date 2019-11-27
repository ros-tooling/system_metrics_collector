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

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
constexpr const char PROC_STAT_FILE[] = "/proc/meminfo";
constexpr const char MEM_TOTAL[] = "MemTotal:";
constexpr const char MEM_AVAILABLE[] = "MemAvailable:";
constexpr const char EMPTY_FILE[] = "";
constexpr const int INVALID_MEMORY_SAMPLE = -1;
}  // namespace

namespace system_metrics_collector {


std::string readFileToString(const std::string & file_name)
{
  std::ifstream file_to_read(file_name);
  if (!file_to_read.good()) {
    RCUTILS_LOG_ERROR_NAMED("readFileToString", "unable to parse file %s", file_name.c_str());
    return EMPTY_FILE;
  }

  std::string to_return((std::istreambuf_iterator<char>(file_to_read)),
    std::istreambuf_iterator<char>());

  return to_return;
}

double processMemInfoLines(const std::string & lines)
{
  std::istringstream process_lines_stream(lines);
  if (!process_lines_stream.good()) {
    RCUTILS_LOG_ERROR("unable to parse input lines");
    return std::nan("");
  }

  std::string line;

  int total = INVALID_MEMORY_SAMPLE;
  int available = INVALID_MEMORY_SAMPLE;

  std::istringstream parse_line("");  // parse each line from the input
  std::string tlabel;

  while (std::getline(process_lines_stream, line) && process_lines_stream.good()) {
    parse_line.str(line);

    if (!line.compare(0, strlen(MEM_TOTAL), MEM_TOTAL)) {
      parse_line >> tlabel;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("processMemInfoLines", "unable to parse %s label", MEM_TOTAL);
        return std::nan("");
      }

      parse_line >> total;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("processMemInfoLines", "unable to parse %s value", MEM_TOTAL);
        return std::nan("");
      }
    } else if (!line.compare(0, strlen(MEM_AVAILABLE), MEM_AVAILABLE)) {
      std::string tlabel;

      parse_line >> tlabel;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("processMemInfoLines", "unable to parse %s label", MEM_AVAILABLE);
        return std::nan("");
      }

      parse_line >> available;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("processMemInfoLines", "unable to parse %s value", MEM_AVAILABLE);
        return std::nan("");
      }
      break;  // no need to parse other lines after this label
    }
    parse_line.clear();
  }
  const double to_return = static_cast<double>(total - available) / static_cast<double>(total) *
    100.0;
  return total == INVALID_MEMORY_SAMPLE || available == INVALID_MEMORY_SAMPLE ?
         std::nan("") : to_return;
}

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

}  // namespace system_metrics_collector
