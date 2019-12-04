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

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "../../src/system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

namespace
{
constexpr const char MEASUREMENT_TYPE[] = "system_cpu_usage";
constexpr const char PROC_STAT_FILE[] = "/proc/stat";
constexpr const char CPU_LABEL[] = "cpu";
}  // namespace

namespace system_metrics_collector
{

system_metrics_collector::ProcCpuData processStatCpuLine(const std::string & stat_cpu_line)
{
  system_metrics_collector::ProcCpuData parsed_data;

  if (!stat_cpu_line.empty()) {
    if (!stat_cpu_line.compare(0, strlen(CPU_LABEL), CPU_LABEL)) {
      std::istringstream ss(stat_cpu_line);

      if (!ss.good()) {
        return system_metrics_collector::ProcCpuData();
      }
      ss >> parsed_data.cpu_label;

      for (int i = 0;
        i < static_cast<int>(system_metrics_collector::ProcCpuStates::kNumProcCpuStates); ++i)
      {
        if (!ss.good()) {
          return system_metrics_collector::ProcCpuData();
        }
        ss >> parsed_data.times[i];
      }

      return parsed_data;
    }
  }
  return parsed_data;
}

double computeCpuActivePercentage(
  const system_metrics_collector::ProcCpuData & measurement1,
  const system_metrics_collector::ProcCpuData & measurement2)
{
  if (measurement1.isMeasurementEmpty() || measurement2.isMeasurementEmpty()) {
    RCUTILS_LOG_ERROR_NAMED("computeCpuActivePercentage",
      "a measurement was empty, unable to compute cpu percentage");
    return std::nan("");
  }

  const double active_time = measurement2.getActiveTime() - measurement1.getActiveTime();
  const double total_time = (measurement2.getIdleTime() + measurement2.getActiveTime()) -
    (measurement1.getIdleTime() + measurement1.getActiveTime());

  return 100.0 * active_time / total_time;
}

LinuxCpuMeasurementNode::LinuxCpuMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & topic,
  const std::chrono::milliseconds publish_period)
: PeriodicMeasurementNode(name, measurement_period, topic, publish_period),
  last_measurement_()
{}

double LinuxCpuMeasurementNode::periodicMeasurement()
{
  const system_metrics_collector::ProcCpuData current_measurement = makeSingleMeasurement();

  const double cpu_percentage = computeCpuActivePercentage(
    last_measurement_,
    current_measurement);

  last_measurement_ = current_measurement;

  return cpu_percentage;
}

system_metrics_collector::ProcCpuData LinuxCpuMeasurementNode::makeSingleMeasurement()
{
  std::ifstream stat_file(PROC_STAT_FILE);
  if (!stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to open file %s", PROC_STAT_FILE);
    return system_metrics_collector::ProcCpuData();
  }
  std::string line;
  std::getline(stat_file, line);

  if (!stat_file.good()) {
    RCLCPP_ERROR(this->get_logger(), "unable to get cpu line from file");
    return system_metrics_collector::ProcCpuData();
  } else {
    return processStatCpuLine(line);
  }
}

void LinuxCpuMeasurementNode::publishStatisticMessage()
{
  auto msg = generateStatisticMessage(get_name(), MEASUREMENT_TYPE, window_start_,
      now(), getStatisticsResults());
  publisher_->publish(msg);
}

}  // namespace system_metrics_collector
