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

#include <sstream>
#include <string>

#include "proc_pid_cpu_data.hpp"


namespace system_metrics_collector
{

/*static*/ constexpr const uint64_t ProcPidCpuData::kEmptyData;

uint64_t ProcPidCpuData::GetActiveTime() const
{
  return pid_cpu_time;
}

uint64_t ProcPidCpuData::GetTotalTime() const
{
  return total_cpu_time;
}

bool ProcPidCpuData::IsMeasurementEmpty() const
{
  return pid_cpu_time == kEmptyData || total_cpu_time == kEmptyData;
}

std::string ProcPidCpuData::ToString() const
{
  std::stringstream ss;
  ss << "pid_cpu_time=" << pid_cpu_time << ", total_cpu_time=" << total_cpu_time;
  return ss.str();
}

}  // namespace system_metrics_collector
