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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "linux_cpu_measurement_node.hpp"
#include "linux_memory_measurement_node.hpp"
#include "linux_process_cpu_measurement_node.hpp"
#include "linux_process_memory_measurement_node.hpp"

/**
 * Sets a node's logging verbosity to debug.
 *
 * @param node the node to set to debug
 */
void set_node_to_debug(
  const system_metrics_collector::PeriodicMeasurementNode & node)
{
  const auto r = rcutils_logging_set_logger_level(node.get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (r != 0) {
    RCUTILS_LOG_ERROR_NAMED(
      "main", "Unable to set debug logging for the %s node: %s\n",
      node.get_name(),
      rcutils_get_error_string().str);
  }
}

/**
 * Creates and manually executes metric collector nodes.
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using namespace std::chrono_literals;
  const auto cpu_node =
    std::make_shared<system_metrics_collector::LinuxCpuMeasurementNode>("linuxCpuCollector");

  const auto mem_node =
    std::make_shared<system_metrics_collector::LinuxMemoryMeasurementNode>("linuxMemoryCollector");

  const auto process_cpu_node =
    std::make_shared<system_metrics_collector::LinuxProcessCpuMeasurementNode>(
    "linuxProcessCpuCollector");

  const auto process_mem_node =
    std::make_shared<system_metrics_collector::LinuxProcessMemoryMeasurementNode>(
    "linuxProcessMemoryCollector");

  rclcpp::executors::MultiThreadedExecutor ex;

  // manually configure lifecycle nodes in order to activate
  cpu_node->configure();
  mem_node->configure();
  process_cpu_node->configure();
  process_mem_node->configure();

  // once spinning, manual activation allows the collection of data to start automatically
  cpu_node->activate();
  mem_node->activate();
  process_cpu_node->activate();
  process_mem_node->activate();

  set_node_to_debug(*cpu_node);
  set_node_to_debug(*mem_node);
  set_node_to_debug(*process_cpu_node);
  set_node_to_debug(*process_mem_node);

  ex.add_node(cpu_node->get_node_base_interface());
  ex.add_node(mem_node->get_node_base_interface());
  ex.add_node(process_cpu_node->get_node_base_interface());
  ex.add_node(process_mem_node->get_node_base_interface());

  ex.spin();

  // cleanup the lifecycle nodes
  cpu_node->shutdown();
  mem_node->shutdown();
  process_cpu_node->shutdown();
  process_mem_node->shutdown();

  rclcpp::shutdown();

  return 0;
}
