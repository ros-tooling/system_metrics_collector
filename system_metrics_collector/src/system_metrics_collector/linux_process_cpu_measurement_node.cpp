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

#include <sys/sysinfo.h>
#include <sys/types.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>

#include "rcutils/logging_macros.h"

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
  metric_name_(std::to_string(GetPid()) + kMetricName)
{
}

bool LinuxProcessCpuMeasurementNode::SetupStart()
{
  last_measurement_ = ProcPidCpuData();
  return PeriodicMeasurementNode::SetupStart();
}

double LinuxProcessCpuMeasurementNode::PeriodicMeasurement()
{
  const auto current_measurement = MeasurePidCpuTime();

  const auto cpu_percentage = ComputePidCpuActivePercentage(last_measurement_, current_measurement);

  last_measurement_ = current_measurement;

  return cpu_percentage;
}

std::string LinuxProcessCpuMeasurementNode::GetMetricName() const
{
  return metric_name_;
}

}   // namespace system_metrics_collector
