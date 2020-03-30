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
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_process_memory_measurement_node.hpp"
#include "system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"


namespace
{
using lifecycle_msgs::msg::State;

constexpr const char kTestNodeName[] = "test_periodic_node";
constexpr const char kTestStatmLine[] = "2084389 308110 7390 1 0 366785 0\n";
constexpr const int kExpectedProcessMemoryUsed{2084389};
constexpr const std::chrono::seconds kMeasurePeriod{1};
constexpr const std::chrono::seconds kPublishPeriod{10};
}

class TestLinuxProcessMemoryMeasurementNode : public system_metrics_collector::
  LinuxProcessMemoryMeasurementNode
{
public:
  TestLinuxProcessMemoryMeasurementNode(
    const std::string & name,
    const rclcpp::NodeOptions & options)
  : LinuxProcessMemoryMeasurementNode(name, options) {}

  std::string GetMetricName() const override
  {
    return LinuxProcessMemoryMeasurementNode::GetMetricName();
  }

  std::string GetMetricUnit() const override
  {
    return LinuxProcessMemoryMeasurementNode::GetMetricUnit();
  }
};

class LinuxProcessMemoryMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      std::chrono::duration_cast<std::chrono::milliseconds>(kMeasurePeriod).count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      std::chrono::duration_cast<std::chrono::milliseconds>(kPublishPeriod).count());

    test_node_ = std::make_shared<TestLinuxProcessMemoryMeasurementNode>(
      kTestNodeName, options);

    ASSERT_FALSE(test_node_->IsStarted());
    ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_node_->get_current_state().id());

    const auto data = test_node_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_node_->shutdown();
    EXPECT_FALSE(test_node_->IsStarted());
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_node_->get_current_state().id());

    test_node_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxProcessMemoryMeasurementNode> test_node_;
};

TEST(TestLinuxProcessMemoryMeasurement, TestGetProcessUsedMemory) {
  EXPECT_THROW(
    system_metrics_collector::GetProcessUsedMemory(
      test_constants::kGarbageSample), std::ifstream::failure);
  EXPECT_THROW(
    system_metrics_collector::GetProcessUsedMemory(
      test_constants::kEmptySample), std::ifstream::failure);

  const auto ret = system_metrics_collector::GetProcessUsedMemory(kTestStatmLine);
  EXPECT_EQ(kExpectedProcessMemoryUsed, ret);
}

TEST_F(LinuxProcessMemoryMeasurementTestFixture, TestGetMetricName) {
  const auto pid = system_metrics_collector::GetPid();
  ASSERT_EQ(std::to_string(pid) + "_memory_percent_used", test_node_->GetMetricName());
}

TEST_F(LinuxProcessMemoryMeasurementTestFixture, TestGetMetricUnit) {
  const auto pid = system_metrics_collector::GetPid();
  ASSERT_EQ(
    system_metrics_collector::collector_node_constants::kPercentUnitName,
    test_node_->GetMetricUnit());
}
