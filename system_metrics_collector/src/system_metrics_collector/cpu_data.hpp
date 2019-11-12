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
  std::string cpu_label = "empty";  // todo fixme magic string constant
  size_t times[kNumProcCpuStates] = {0};

  size_t getIdleTime() const
  {
    return times[idle] + times[iOWait];
  }

  size_t getActiveTime() const
  {
    return times[user] + times[nice] + times[system] + times[irq] +
           times[softIrq] + times[steal];
  }

  /**
   * Return a pretty printed string of the ProcCpuData struct.
   *
   * @param data the struct to print
   * @return a formatted string of the input struct
   */
  std::string toString()
  {
    std::stringstream ss;
    ss << "cpu_label=" << cpu_label <<
      ", user=" << times[user] <<
      ", nice=" << times[nice] <<
      ", system=" << times[system] <<
      ", idle=" << times[idle] <<
      ", iOWait=" << times[iOWait] <<
      ", irq=" << times[irq] <<
      ", softIrq=" << times[softIrq] <<
      ", steal=" << times[steal];
    return ss.str();
  }

  bool isMeasurementEmpty() const
  {
    return cpu_label == "empty";  //  todo fixme magic string constant
  }
};

#endif  // SYSTEM_METRICS_COLLECTOR__CPU_DATA_HPP_
