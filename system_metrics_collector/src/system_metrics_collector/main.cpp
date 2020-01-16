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


#include <functional>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "../../src/system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/linux_process_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/linux_process_memory_measurement_node.hpp"


void set_node_to_debug(
  const system_metrics_collector::PeriodicMeasurementNode * node,
  const char * node_type)
{
  const auto r = rcutils_logging_set_logger_level(node->get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (r != 0) {
    RCUTILS_LOG_ERROR_NAMED("main", "Unable to set debug logging for the %s node: %s\n", node_type,
      rcutils_get_error_string().str);
  }
}

/**
 * This is current a "test" main in order to manually test the measurement nodes.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using namespace std::chrono_literals;
  const auto cpu_node = std::make_shared<system_metrics_collector::LinuxCpuMeasurementNode>(
    "linuxCpuCollector");

  const auto mem_node = std::make_shared<system_metrics_collector::LinuxMemoryMeasurementNode>(
    "linuxMemoryCollector");

  const auto process_cpu_node =
    std::make_shared<system_metrics_collector::LinuxProcessCpuMeasurementNode>(
    "linuxProcessCpuCollector");

  const auto process_mem_node =
    std::make_shared<system_metrics_collector::LinuxProcessMemoryMeasurementNode>(
    "linuxProcessMemoryCollector");

  rclcpp::executors::MultiThreadedExecutor ex;
  cpu_node->Start();
  mem_node->Start();
  process_cpu_node->Start();
  process_mem_node->Start();

  set_node_to_debug(cpu_node.get(), "cpu");
  set_node_to_debug(mem_node.get(), "memory");
  set_node_to_debug(process_cpu_node.get(), "process cpu");
  set_node_to_debug(process_mem_node.get(), "process memory");

  ex.add_node(cpu_node);
  ex.add_node(mem_node);
  ex.add_node(process_cpu_node);
  ex.add_node(process_mem_node);
  ex.spin();

  rclcpp::shutdown();

  cpu_node->Stop();
  mem_node->Stop();
  process_cpu_node->Stop();
  process_mem_node->Stop();

  return 0;
}
