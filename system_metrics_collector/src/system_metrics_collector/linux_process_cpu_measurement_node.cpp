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

constexpr const char PROC_STAT_FILE[] = "/proc/stat";
constexpr const char PROC[] = "/proc/";
constexpr const char STAT[] = "/stat";
constexpr const char METRIC_NAME[] = "_cpu_percent_used";

}  // namespace

namespace system_metrics_collector
{

LinuxProcessCpuMeasurementNode::LinuxProcessCpuMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period),
  pid_(std::to_string(getPid())),
  file_to_read_(PROC + pid_ + STAT)
{
}

bool LinuxProcessCpuMeasurementNode::setupStart()
{
  last_process_measurement_ = ProcPidCpuData();
  last_system_measurement_ = ProcCpuData();
  return PeriodicMeasurementNode::setupStart();
}

double LinuxProcessCpuMeasurementNode::periodicMeasurement()
{
  ProcPidCpuData current_process_measurement;
  ProcCpuData current_system_measurement;

  std::tie(current_process_measurement, current_system_measurement) = makeSingleMeasurement();

  const auto cpu_percentage = computePidCpuActivePercentage(
    last_process_measurement_, last_system_measurement_, current_process_measurement,
    current_system_measurement);

  last_process_measurement_ = current_process_measurement;
  last_system_measurement_ = current_system_measurement;

  return cpu_percentage;
}

std::tuple<ProcPidCpuData, ProcCpuData> LinuxProcessCpuMeasurementNode::makeSingleMeasurement()
{
  std::ifstream pid_stat_file{file_to_read_};
  if (!pid_stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to open file %s", file_to_read_.c_str());
    return std::tuple<ProcPidCpuData, ProcCpuData>();
  }

  std::ifstream stat_file{PROC_STAT_FILE};
  if (!stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to open file %s", PROC_STAT_FILE);
    return std::tuple<ProcPidCpuData, ProcCpuData>();
  }

  std::string pid_line;
  std::getline(pid_stat_file, pid_line);
  std::string sys_line;
  std::getline(stat_file, sys_line);

  if (pid_stat_file.good() && stat_file.good()) {
    return std::make_tuple(processPidStatCpuLine(pid_line), processStatCpuLine(sys_line));
  } else {
    RCLCPP_ERROR(this->get_logger(), "unable to get cpu line from file");
    return std::tuple<ProcPidCpuData, ProcCpuData>();
  }
}

std::string LinuxProcessCpuMeasurementNode::getMetricName() const
{
  return pid_ + METRIC_NAME;
}

}   // namespace system_metrics_collector
