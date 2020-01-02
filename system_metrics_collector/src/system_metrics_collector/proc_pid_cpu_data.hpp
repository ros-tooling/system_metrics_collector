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

#ifndef SYSTEM_METRICS_COLLECTOR__PROC_PID_CPU_DATA_HPP_
#define SYSTEM_METRICS_COLLECTOR__PROC_PID_CPU_DATA_HPP_

#include <array>
#include <limits>
#include <sstream>
#include <string>

namespace system_metrics_collector
{

class ProcPidCpuData
{
public:
  constexpr ProcPidCpuData() = default;
  constexpr ProcPidCpuData(uint64_t pid_time, uint64_t total_time)
  : pid_cpu_time(pid_time), total_cpu_time(total_time) {}

  static constexpr const uint64_t kEmptyData = std::numeric_limits<uint64_t>::max();

  /**
   * Returns the active time for this data set
   * @return the active time
   */
  uint64_t GetActiveTime() const;

  /**
   * Returns the total elapsed system time for this data set
   * @return the total elapsed system time
   */
  uint64_t GetTotalTime() const;

  /**
   * Returns true if the struct is not populated with any useful data.
   * This indicates a parsing error.
   *
   * @return true if empty (no valid data), false otherwise.
   */
  bool IsMeasurementEmpty() const;

  /**
   * Returns a pretty printed string of the ProcPidCpuData struct.
   *
   * @param data the struct to print
   * @return a formatted string of the input struct
   */
  std::string ToString() const;

  /**
   * Stored CPU data: user and system mode cpu time for the process in nanoseconds
   */
  uint64_t pid_cpu_time{kEmptyData};
  /**
   * Stored CPU data: wall clock time (cpu time of entire system) in nanoseconds
   */
  uint64_t total_cpu_time{kEmptyData};
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__PROC_PID_CPU_DATA_HPP_
