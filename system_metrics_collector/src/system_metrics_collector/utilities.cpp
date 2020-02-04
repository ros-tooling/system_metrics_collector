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

#include <chrono>
#include <cmath>
#include <ctime>
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

constexpr const char kCpuLabel[] = "cpu";
constexpr const char kMemTotal[] = "MemTotal:";
constexpr const char kMemAvailable[] = "MemAvailable:";
constexpr const char kEmptyFile[] = "";
constexpr const int kInvalidMemorySample = -1;

uint64_t TimespecToNanoseconds(const timespec & ts)
{
  auto duration = std::chrono::seconds{ts.tv_sec} + std::chrono::nanoseconds{ts.tv_nsec};
  return std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
}

}  // namespace

std::string ReadFileToString(const std::string & file_name)
{
  std::ifstream file_to_read{file_name};
  if (!file_to_read.good()) {
    RCUTILS_LOG_ERROR_NAMED("ReadFileToString", "unable to parse file: %s", file_name.c_str());
    return kEmptyFile;
  }

  std::string to_return((std::istreambuf_iterator<char>(file_to_read)),
    std::istreambuf_iterator<char>());

  return to_return;
}

ProcCpuData ProcessStatCpuLine(const std::string & stat_cpu_line)
{
  ProcCpuData parsed_data;

  if (!stat_cpu_line.empty()) {
    if (stat_cpu_line.compare(0, strlen(kCpuLabel), kCpuLabel) == 0) {
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

ProcPidCpuData MeasurePidCpuTime()
{
  static const int kNumCpus = sysconf(_SC_NPROCESSORS_ONLN);
  timespec process_time{};
  timespec monotonic_time{};
  ProcPidCpuData data;

  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &process_time) != 0) {
    RCUTILS_LOG_ERROR_NAMED("MeasurePidCpuTime", "unable to get process cpu time");
    return data;
  }
  if (clock_gettime(CLOCK_MONOTONIC_RAW, &monotonic_time) != 0) {
    RCUTILS_LOG_ERROR_NAMED("MeasurePidCpuTime", "unable to get monotonic cpu time");
    return data;
  }

  data.pid_cpu_time = TimespecToNanoseconds(process_time);
  data.total_cpu_time = kNumCpus * TimespecToNanoseconds(monotonic_time);

  return data;
}

double ComputeCpuActivePercentage(
  const ProcCpuData & measurement1,
  const ProcCpuData & measurement2)
{
  if (measurement1.IsMeasurementEmpty() || measurement2.IsMeasurementEmpty()) {
    RCUTILS_LOG_ERROR_NAMED(
      "ComputeCpuActivePercentage",
      "a measurement was empty, unable to compute cpu percentage");
    return std::nan("");
  }

  const double active_time = measurement2.GetActiveTime() - measurement1.GetActiveTime();
  const double total_time = (measurement2.GetIdleTime() + measurement2.GetActiveTime()) -
    (measurement1.GetIdleTime() + measurement1.GetActiveTime());

  return 100.0 * active_time / total_time;
}

double ComputePidCpuActivePercentage(
  const ProcPidCpuData & measurement1,
  const ProcPidCpuData & measurement2)
{
  if (measurement1.IsMeasurementEmpty() || measurement2.IsMeasurementEmpty()) {
    RCUTILS_LOG_ERROR_NAMED(
      "ComputePidCpuActivePercentage",
      "a measurement was empty, unable to compute pid cpu percentage");
    return std::nan("");
  }

  const double active_time = measurement2.GetActiveTime() - measurement1.GetActiveTime();
  const double total_time = measurement2.GetTotalTime() - measurement1.GetTotalTime();

  return 100.0 * active_time / total_time;
}

double ProcessMemInfoLines(const std::string & lines)
{
  std::istringstream process_lines_stream(lines);
  if (!process_lines_stream.good()) {
    RCUTILS_LOG_ERROR("unable to parse input lines");
    return std::nan("");
  }

  std::string line;

  int total = kInvalidMemorySample;
  int available = kInvalidMemorySample;

  std::istringstream parse_line("");      // parse each line from the input
  std::string tlabel;

  while (std::getline(process_lines_stream, line) && process_lines_stream.good()) {
    parse_line.str(line);

    if (line.compare(0, strlen(kMemTotal), kMemTotal) == 0) {
      parse_line >> tlabel;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("ProcessMemInfoLines", "unable to parse %s label", kMemTotal);
        return std::nan("");
      }

      parse_line >> total;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("ProcessMemInfoLines", "unable to parse %s value", kMemTotal);
        return std::nan("");
      }
    } else if (line.compare(0, strlen(kMemAvailable), kMemAvailable) == 0) {
      std::string tlabel;

      parse_line >> tlabel;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("ProcessMemInfoLines", "unable to parse %s label", kMemAvailable);
        return std::nan("");
      }

      parse_line >> available;
      if (!parse_line.good()) {
        RCUTILS_LOG_ERROR_NAMED("ProcessMemInfoLines", "unable to parse %s value", kMemAvailable);
        return std::nan("");
      }
      break;      // no need to parse other lines after this label
    }
    parse_line.clear();
  }
  const double to_return = static_cast<double>(total - available) / static_cast<double>(total) *
    100.0;
  return total == kInvalidMemorySample || available == kInvalidMemorySample ?
         std::nan("") : to_return;
}

int GetPid()
{
  return getpid();
}

}  // namespace system_metrics_collector
