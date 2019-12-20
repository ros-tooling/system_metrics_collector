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

#include <unistd.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <streambuf>
#include <string>

#include "utilities.hpp"
#include "rcutils/logging_macros.h"

namespace system_metrics_collector
{

namespace
{

constexpr const char CPU_LABEL[] = "cpu";
constexpr const char MEM_TOTAL[] = "MemTotal:";
constexpr const char MEM_AVAILABLE[] = "MemAvailable:";
constexpr const char EMPTY_FILE[] = "";
constexpr const int INVALID_MEMORY_SAMPLE = -1;

double computeCpuTotalTime(const ProcCpuData measurement1, const ProcCpuData measurement2)
{
  const double total_time = (measurement2.getIdleTime() + measurement2.getActiveTime()) -
    (measurement1.getIdleTime() + measurement1.getActiveTime());
  return total_time;
}

}  // namespace

std::string readFileToString(const std::string & file_name)
{
  std::ifstream file_to_read(file_name);
  if (!file_to_read.good()) {
    RCUTILS_LOG_ERROR_NAMED("readFileToString", "unable to parse file: %s", file_name.c_str());
    return EMPTY_FILE;
  }

  std::string to_return((std::istreambuf_iterator<char>(file_to_read)),
    std::istreambuf_iterator<char>());

  return to_return;
}

ProcCpuData processStatCpuLine(const std::string & stat_cpu_line)
{
  ProcCpuData parsed_data;

  if (!stat_cpu_line.empty()) {
    if (!stat_cpu_line.compare(0, strlen(CPU_LABEL), CPU_LABEL)) {
      std::istringstream ss(stat_cpu_line);

      if (!ss.good()) {
        return ProcCpuData();
      }
      ss >> parsed_data.cpu_label;

      for (int i = 0;
        i < static_cast<int>(ProcCpuStates::kNumProcCpuStates); ++i)
      {
        if (!ss.good()) {
          return ProcCpuData();
        }
        ss >> parsed_data.times[i];
      }
      return parsed_data;
    }
  }
  return parsed_data;
}

ProcPidCpuData processPidStatCpuLine(const std::string & stat_cpu_line)
{
  ProcPidCpuData parsed_data;

  if (!stat_cpu_line.empty()) {
    std::istringstream ss(stat_cpu_line);
    ss.exceptions(std::ios::failbit | std::ios::badbit);

    try {
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // pid
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // comm
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // state
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // ppid
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // pgrp
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // session
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // tty_nr
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // tpgid
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // flags
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // minflt
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // cminflt
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // majflt
      ss.ignore(std::numeric_limits<std::streamsize>::max(), ' ');  // cmajflt
      ss >> parsed_data.utime;
      ss >> parsed_data.stime;
    } catch (std::ifstream::failure & e) {
      RCUTILS_LOG_ERROR_NAMED("processPidStatCpuLine",
        "unable to parse string for process cpu usage");
      return ProcPidCpuData();
    }
  }

  return parsed_data;
}

double computeCpuActivePercentage(
  const ProcCpuData & measurement1,
  const ProcCpuData & measurement2)
{
  if (measurement1.isMeasurementEmpty() || measurement2.isMeasurementEmpty()) {
    RCUTILS_LOG_ERROR_NAMED("computeCpuActivePercentage",
      "a measurement was empty, unable to compute cpu percentage");
    return std::nan("");
  }

  const double active_time = measurement2.getActiveTime() - measurement1.getActiveTime();
  const double total_time = computeCpuTotalTime(measurement1, measurement2);

  return 100.0 * active_time / total_time;
}

double computePidCpuActivePercentage(
  const ProcPidCpuData & process_measurement1,
  const ProcCpuData & system_measurement1,
  const ProcPidCpuData & process_measurement2,
  const ProcCpuData & system_measurement2)
{
  if (process_measurement1.isMeasurementEmpty() || system_measurement1.isMeasurementEmpty() ||
    process_measurement2.isMeasurementEmpty() || system_measurement2.isMeasurementEmpty())
  {
    RCUTILS_LOG_ERROR_NAMED("computePidCpuActivePercentage",
      "a measurement was empty, unable to compute cpu percentage");
    return std::nan("");
  }

  const double active_time = process_measurement2.getActiveTime() -
    process_measurement1.getActiveTime();
  const double total_time = computeCpuTotalTime(system_measurement1, system_measurement2);

  return 100.0 * active_time / total_time;
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

  std::istringstream parse_line("");      // parse each line from the input
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
      break;      // no need to parse other lines after this label
    }
    parse_line.clear();
  }
  const double to_return = static_cast<double>(total - available) / static_cast<double>(total) *
    100.0;
  return total == INVALID_MEMORY_SAMPLE || available == INVALID_MEMORY_SAMPLE ?
         std::nan("") : to_return;
}

int getPid()
{
  return getpid();
}

}  // namespace system_metrics_collector
