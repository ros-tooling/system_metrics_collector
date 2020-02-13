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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"

#include "test_constants.hpp"

namespace
{
constexpr const char kSystemMetricsCollectorLibName[] = "libsystem_metrics_collector.so";
constexpr const std::array<const char *, 2> kExpectedClassNames = {
  "system_metrics_collector::LinuxProcessCpuMeasurementNode",
  "system_metrics_collector::LinuxProcessMemoryMeasurementNode"
};
}

bool IsExpectedClassName(const std::string & class_name)
{
  const auto result = std::find_if(
    kExpectedClassNames.cbegin(), kExpectedClassNames.cend(),
    [&class_name](const char * expected_class_name) {
      return class_name.find(expected_class_name) != std::string::npos;
    });
  return result != kExpectedClassNames.cend();
}

TEST(TestComposeableNodes, DlopenTest)
{
  rclcpp::init(0, nullptr);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  const auto loader = std::make_unique<class_loader::ClassLoader>(kSystemMetricsCollectorLibName);
  const auto class_names = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
  ASSERT_EQ(kExpectedClassNames.size(), class_names.size());

  std::vector<rclcpp_components::NodeInstanceWrapper> node_wrappers;
  for (const auto & class_name : class_names) {
    ASSERT_TRUE(IsExpectedClassName(class_name));
    const auto node_factory = loader->createInstance<rclcpp_components::NodeFactory>(class_name);
    auto wrapper = node_factory->create_node_instance(options);
    exec.add_node(wrapper.get_node_base_interface());
    node_wrappers.push_back(wrapper);
  }

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  exec.spin_until_future_complete(dummy_future, test_constants::kTestDuration);

  for (auto & wrapper : node_wrappers) {
    exec.remove_node(wrapper.get_node_base_interface());
  }
  rclcpp::shutdown();
}
