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

#include "linux_process_cpu_measurement_node.hpp"

#include <chrono>
#include <string>

#include "../../src/system_metrics_collector/utilities.hpp"


namespace
{

constexpr const char kProcStatFile[] = "/proc/stat";
constexpr const char kProc[] = "/proc/";
constexpr const char kStat[] = "/stat";
constexpr const char kMetricName[] = "_cpu_percent_used";

}  // namespace

namespace system_metrics_collector
{

LinuxProcessCpuMeasurementNode::LinuxProcessCpuMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period),
  pid_(std::to_string(GetPid()))
{
}

bool LinuxProcessCpuMeasurementNode::SetupStart()
{
  return false;
}

double LinuxProcessCpuMeasurementNode::PeriodicMeasurement()
{
  return 0.0;
}

std::string LinuxProcessCpuMeasurementNode::GetMetricName() const
{
  return pid_ + kMetricName;
}

}   // namespace system_metrics_collector
