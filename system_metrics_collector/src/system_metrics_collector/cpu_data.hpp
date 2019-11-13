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

#ifndef SYSTEM_METRICS_COLLECTOR__CPU_DATA_HPP_
#define SYSTEM_METRICS_COLLECTOR__CPU_DATA_HPP_

#include <string>
#include <sstream>

/**
 * Struct containing data parsed from a /proc/stat cpu line
 */
struct ProcCpuData
{
  static constexpr const char EMPTY_LABEL[] = "empty";
  /**
   * Enum representing each item in a /proc/stat cpu line
   */
  enum ProcCpuStates
  {
    user = 0,
    nice,
    system,
    idle,
    iOWait,
    irq,
    softIrq,
    steal,
    kNumProcCpuStates
  };

  /**
   * The cpu label of the line parsed
   */
  std::string cpu_label = EMPTY_LABEL;
  size_t times[kNumProcCpuStates] = {0};

  /**
   * Return the idle time
   * @return the idle time for this data set
   */
  size_t getIdleTime() const;

  /**
   * Return the active time
   * @return the active time for this data set
   */
  size_t getActiveTime() const;

  /**
   * Return a pretty printed string of the ProcCpuData struct.
   *
   * @param data the struct to print
   * @return a formatted string of the input struct
   */
  std::string toString() const;

  /**
   * Return true if the struct is not populated with any useful data.
   * This indicates a parsing error.
   *
   * @return true if empty (no valid data), false otherwise.
   */
  bool isMeasurementEmpty() const;
};

#endif  // SYSTEM_METRICS_COLLECTOR__CPU_DATA_HPP_
