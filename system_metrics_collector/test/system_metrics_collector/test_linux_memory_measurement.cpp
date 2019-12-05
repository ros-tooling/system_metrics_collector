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

#include <chrono>
#include <iostream>
#include <string>
#include <memory>
#include <mutex>

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

namespace
{
constexpr const auto TEST_PERIOD{std::chrono::milliseconds(50)};
}  // namespace

class TestLinuxMemoryMeasurementNode : public system_metrics_collector::LinuxMemoryMeasurementNode
{
public:
  TestLinuxMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic)
  : LinuxMemoryMeasurementNode(name, measurement_period, publishing_topic, INVALID_PUBLISH_WINDOW)
  {}
  virtual ~TestLinuxMemoryMeasurementNode() = default;

  double periodicMeasurement() override
  {
    // override to avoid calling methods involved in file i/o
    return system_metrics_collector::processMemInfoLines(test_string_);
  }

  void setTestString(std::string & test_string)
  {
    test_string_ = test_string;
  }

private:
  std::string test_string_{""};
};

class LinuxMemoryMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_measure_linux_memory = std::make_shared<TestLinuxMemoryMeasurementNode>(
      "test_periodic_node",
      TEST_PERIOD, "test_topic");

    ASSERT_FALSE(test_measure_linux_memory->isStarted());

    const moving_average_statistics::StatisticData data =
      test_measure_linux_memory->getStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_memory->stop();
    test_measure_linux_memory.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxMemoryMeasurementNode> test_measure_linux_memory;
};

TEST_F(LinuxMemoryMeasurementTestFixture, testManualMeasurement) {
  double mem_used_percentage = test_measure_linux_memory->periodicMeasurement();
  ASSERT_TRUE(std::isnan(mem_used_percentage));

  auto s = std::string(test_constants::FULL_SAMPLE);
  test_measure_linux_memory->setTestString(s);

  mem_used_percentage = test_measure_linux_memory->periodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::MEMORY_USED_PERCENTAGE, mem_used_percentage);
}
