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

#include "../../src/system_metrics_collector/linux_cpu_measurement_node.hpp"

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

  auto node = std::make_shared<LinuxCpuMeasurementNode>(
    "linuxCpuCollector",
    std::chrono::milliseconds(1000),
    "not_publishing_yet",
    std::chrono::milliseconds(1000 * 60));

  rclcpp::executors::SingleThreadedExecutor ex;
  node->start();

  int code = 0;
  auto r = rcutils_logging_set_logger_level(node->get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

  if (r != 0) {
    std::cout << "Unabled to set debug logging!" << std::endl;
    code = 1;
  } else {
    ex.add_node(node);
    ex.spin();
  }


  rclcpp::shutdown();
  node->stop();
  return code;
}
