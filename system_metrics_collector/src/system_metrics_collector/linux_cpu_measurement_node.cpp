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

#include <cmath>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "constants.hpp"
#include "linux_cpu_measurement_node.hpp"
#include "periodic_measurement_node.hpp"
#include "utilities.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

using statistics_msgs::msg::MetricsMessage;

namespace
{
constexpr const char kMeasurementType[] = "system_cpu_percent_used";
constexpr const char kProcStatFile[] = "/proc/stat";
}  // namespace

namespace system_metrics_collector
{

LinuxCpuMeasurementNode::LinuxCpuMeasurementNode(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: PeriodicMeasurementNode{name, options}
{
}

bool LinuxCpuMeasurementNode::SetupStart()
{
  last_measurement_ = ProcCpuData();
  return PeriodicMeasurementNode::SetupStart();
}

double LinuxCpuMeasurementNode::PeriodicMeasurement()
{
  const system_metrics_collector::ProcCpuData current_measurement = MakeSingleMeasurement();

  const auto cpu_percentage = ComputeCpuActivePercentage(
    last_measurement_,
    current_measurement);

  last_measurement_ = current_measurement;

  return cpu_percentage;
}

system_metrics_collector::ProcCpuData LinuxCpuMeasurementNode::MakeSingleMeasurement()
{
  std::ifstream stat_file{kProcStatFile};
  if (!stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to open file %s", kProcStatFile);
    return system_metrics_collector::ProcCpuData();
  }
  std::string line;
  std::getline(stat_file, line);

  if (!stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to get cpu line from file");
    return system_metrics_collector::ProcCpuData();
  } else {
    return ProcessStatCpuLine(line);
  }
}

std::string LinuxCpuMeasurementNode::GetMetricName() const
{
  return kMeasurementType;
}

std::string LinuxCpuMeasurementNode::GetMetricUnit() const
{
  return collector_node_constants::kPercentUnitName;
}

}  // namespace system_metrics_collector
