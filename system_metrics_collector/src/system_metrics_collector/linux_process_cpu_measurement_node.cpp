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

#include "rclcpp_components/register_node_macro.hpp"
#include "rcutils/logging_macros.h"

#include "constants.hpp"

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
  const rclcpp::NodeOptions & options)
: PeriodicMeasurementNode{name, options},
  metric_name_{std::to_string(GetPid()) + kMetricName}
{
}

LinuxProcessCpuMeasurementNode::LinuxProcessCpuMeasurementNode(
  const rclcpp::NodeOptions & options)
: LinuxProcessCpuMeasurementNode{"linux_process_cpu_collector", options}
{
  configure();
  activate();
}

bool LinuxProcessCpuMeasurementNode::SetupStart()
{
  last_measurement_ = ProcPidCpuData();
  return PeriodicMeasurementNode::SetupStart();
}

double LinuxProcessCpuMeasurementNode::PeriodicMeasurement()
{
  const auto current_measurement = MakeSingleMeasurement();

  const auto cpu_percentage = ComputePidCpuActivePercentage(last_measurement_, current_measurement);

  last_measurement_ = current_measurement;

  return cpu_percentage;
}

ProcPidCpuData LinuxProcessCpuMeasurementNode::MakeSingleMeasurement()
{
  return MeasurePidCpuTime();
}

std::string LinuxProcessCpuMeasurementNode::GetMetricName() const
{
  return metric_name_;
}

std::string LinuxProcessCpuMeasurementNode::GetMetricUnit() const
{
  return collector_node_constants::kPercentUnitName;
}

}   // namespace system_metrics_collector


// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(system_metrics_collector::LinuxProcessCpuMeasurementNode);
