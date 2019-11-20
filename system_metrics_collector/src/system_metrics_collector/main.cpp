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

  auto cpu_node = std::make_shared<LinuxCpuMeasurementNode>(
    "linuxCpuCollector",
    std::chrono::milliseconds(1000),
    "not_publishing_yet",
    std::chrono::milliseconds(1000 * 60));

  auto mem_node = std::make_shared<LinuxMemoryMeasurementNode>(
    "linuxMemoryCollector",
    std::chrono::milliseconds(1000),
    "not_publishing_yet",
    std::chrono::milliseconds(1000 * 60));

  rclcpp::executors::MultiThreadedExecutor ex;
  cpu_node->start();
  mem_node->start();

  auto r = rcutils_logging_set_logger_level(cpu_node->get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
  if (r != 0) {
    RCUTILS_LOG_ERROR_NAMED("main", "Unable to set debug logging for the cpu node");
  }

  r = rcutils_logging_set_logger_level(mem_node->get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  if (r != 0) {
    RCUTILS_LOG_ERROR_NAMED("main", "Unable to set debug logging for the memory node");
  }

  ex.add_node(cpu_node);
  ex.add_node(mem_node);
  ex.spin();

  rclcpp::shutdown();
  cpu_node->stop();
  mem_node->stop();
  return r;
}
