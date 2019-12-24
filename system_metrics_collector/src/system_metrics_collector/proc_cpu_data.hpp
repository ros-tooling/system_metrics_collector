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

#ifndef SYSTEM_METRICS_COLLECTOR__PROC_CPU_DATA_HPP_
#define SYSTEM_METRICS_COLLECTOR__PROC_CPU_DATA_HPP_

#include <array>
#include <sstream>
#include <string>

namespace system_metrics_collector
{

/**
 * Enum representing each item in a /proc/stat cpu line
 */
enum class ProcCpuStates
{
  kUser = 0,
  kNice,
  kSystem,
  kIdle,
  kIOWait,
  kIrq,
  kSoftIrq,
  kSteal,
  kNumProcCpuStates
};

/**
 * Struct containing data parsed from a /proc/stat cpu line
 */
class ProcCpuData
{
public:
  ProcCpuData() = default;
  virtual ~ProcCpuData() = default;

  static constexpr const char kEmptyLabel[] = "empty";

  /**
   * Return the idle time
   * @return the idle time for this data set
   */
  uint64_t GetIdleTime() const;

  /**
   * Return the active time
   * @return the active time for this data set
   */
  uint64_t GetActiveTime() const;

  /**
   * Return a pretty printed string of the ProcCpuData struct.
   *
   * @param data the struct to print
   * @return a formatted string of the input struct
   */
  std::string ToString() const;

  /**
   * Return true if the struct is not populated with any useful data.
   * This indicates a parsing error.
   *
   * @return true if empty (no valid data), false otherwise.
   */
  bool IsMeasurementEmpty() const;

  /**
   * The cpu label of the line parsed
   */
  std::string cpu_label{kEmptyLabel};

  /**
   * Array contained the parsed CPU data, where each index
   * of ProcCpuStates contains its labeled data.
   */
  std::array<uint64_t, static_cast<int>(ProcCpuStates::kNumProcCpuStates)> times{};
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__PROC_CPU_DATA_HPP_
