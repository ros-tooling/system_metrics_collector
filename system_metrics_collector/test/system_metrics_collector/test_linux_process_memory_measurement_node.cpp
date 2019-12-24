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

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <fstream>
#include <string>

#include "../../src/system_metrics_collector/linux_process_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

namespace
{
constexpr const char kTestStatmLine[] = "2084389 308110 7390 1 0 366785 0\n";
}

class TestLinuxProcessMemoryMeasurementNode : public system_metrics_collector::
  LinuxProcessMemoryMeasurementNode
{
public:
  TestLinuxProcessMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period)
  : LinuxProcessMemoryMeasurementNode(name, measurement_period, topic, publish_period) {}

  std::string GetMetricName() const override
  {
    return LinuxProcessMemoryMeasurementNode::GetMetricName();
  }
};

class LinuxProcessMemoryMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    using namespace std::chrono_literals;

    test_node = std::make_shared<TestLinuxProcessMemoryMeasurementNode>(
      "test_periodic_node",
      1s,
      "test_topic",
      10s);

    ASSERT_FALSE(test_node->IsStarted());

    const moving_average_statistics::StatisticData data =
      test_node->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_node->Stop();
    ASSERT_FALSE(test_node->IsStarted());
    test_node.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxProcessMemoryMeasurementNode> test_node;
};


TEST(TestLinuxProcessMemoryMeasurement, TestGetProcessUsedMemory) {
  EXPECT_THROW(system_metrics_collector::GetProcessUsedMemory(
      test_constants::kGarbageSample), std::ifstream::failure);
  EXPECT_THROW(system_metrics_collector::GetProcessUsedMemory(
      test_constants::kEmptySample), std::ifstream::failure);

  const auto ret = system_metrics_collector::GetProcessUsedMemory(kTestStatmLine);
  EXPECT_EQ(2084389, ret);
}

TEST_F(LinuxProcessMemoryMeasurementTestFixture, TestGetMetricName) {
  const auto pid = system_metrics_collector::GetPid();

  ASSERT_EQ(std::to_string(pid) + "_memory_percent_used", test_node->GetMetricName());
}
