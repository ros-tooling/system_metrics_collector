// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"

namespace
{
constexpr const char kStatisticsTopicName[] = "system_metrics";
constexpr const std::chrono::seconds kDefaultCollectPeriod{1};
constexpr const std::chrono::minutes kDefaultPublishPeriod{1};
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  using namespace std::chrono_literals;
  const auto mem_node = std::make_shared<system_metrics_collector::LinuxMemoryMeasurementNode>(
    "linuxMemoryCollector",
    kDefaultCollectPeriod,
    kStatisticsTopicName,
    kDefaultPublishPeriod);

  rclcpp::executors::MultiThreadedExecutor ex;
  mem_node->Start();

  {
    const auto r =
      rcutils_logging_set_logger_level(mem_node->get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    if (r != 0) {
      RCUTILS_LOG_ERROR_NAMED("linux_mem_main",
        "Unable to set debug logging for the memory node: %s\n",
        rcutils_get_error_string().str);
    }
  }

  ex.add_node(mem_node);
  ex.spin();

  rclcpp::shutdown();
  mem_node->Stop();
  return 0;
}
