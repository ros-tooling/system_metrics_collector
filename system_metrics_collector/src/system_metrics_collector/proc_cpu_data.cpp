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

#include <string>
#include <sstream>

#include "proc_cpu_data.hpp"

namespace system_metrics_collector
{

/*static*/ constexpr const char ProcCpuData::kEmptyLabel[];

uint64_t ProcCpuData::GetIdleTime() const
{
  return times[static_cast<int>(ProcCpuStates::kIdle)] +
         times[static_cast<int>(ProcCpuStates::kIOWait)];
}

uint64_t ProcCpuData::GetActiveTime() const
{
  return times[static_cast<int>(ProcCpuStates::kUser)] +
         times[static_cast<int>(ProcCpuStates::kNice)] +
         times[static_cast<int>(ProcCpuStates::kSystem)] +
         times[static_cast<int>(ProcCpuStates::kIrq)] +
         times[static_cast<int>(ProcCpuStates::kSoftIrq)] +
         times[static_cast<int>(ProcCpuStates::kSteal)];
}

std::string ProcCpuData::ToString() const
{
  std::stringstream ss;
  ss << "cpu_label=" << cpu_label <<
    ", user=" << times[static_cast<int>(ProcCpuStates::kUser)] <<
    ", nice=" << times[static_cast<int>(ProcCpuStates::kNice)] <<
    ", system=" << times[static_cast<int>(ProcCpuStates::kSystem)] <<
    ", idle=" << times[static_cast<int>(ProcCpuStates::kIdle)] <<
    ", iOWait=" << times[static_cast<int>(ProcCpuStates::kIOWait)] <<
    ", irq=" << times[static_cast<int>(ProcCpuStates::kIrq)] <<
    ", softIrq=" << times[static_cast<int>(ProcCpuStates::kSoftIrq)] <<
    ", steal=" << times[static_cast<int>(ProcCpuStates::kSteal)];
  return ss.str();
}

bool ProcCpuData::IsMeasurementEmpty() const
{
  return cpu_label == ProcCpuData::kEmptyLabel;
}

}  // namespace system_metrics_collector
