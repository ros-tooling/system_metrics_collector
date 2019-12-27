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
#include <tuple>
#include <unordered_map>

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "../../src/system_metrics_collector/linux_process_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"
#include "test_utilities.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using moving_average_statistics::StatisticData;

namespace
{
constexpr const char kTestNodeName[] = "test_measure_linux_process_cpu";
constexpr const char kTestTopic[] = "test_process_cpu_measure_topic";
}

class TestLinuxProcessCpuMeasurementNode : public system_metrics_collector::
  LinuxProcessCpuMeasurementNode
{
public:
  TestLinuxProcessCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & topic,
    const std::chrono::milliseconds publish_period)
  : LinuxProcessCpuMeasurementNode(name, measurement_period, topic, publish_period) {}

  double PeriodicMeasurement() override
  {
    LinuxProcessCpuMeasurementNode::PeriodicMeasurement();
  }

  std::string GetMetricName() const override
  {
    return LinuxProcessCpuMeasurementNode::GetMetricName();
  }
};

class TestReceiveProcessCpuMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveProcessCpuMeasurementNode(
    const std::string & name,
    const std::string & metric)
  : rclcpp::Node(name)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription_ = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(kTestTopic, 10 /*history_depth*/, callback);
  }

private:
  void MetricsMessageCallback(const MetricsMessage & msg) const
  {
  }

  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
};

class LinuxProcessCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    rclcpp::init(0, nullptr);

    test_node_ = std::make_shared<TestLinuxProcessCpuMeasurementNode>(kTestNodeName,
        test_constants::kMeasurePeriod, kTestTopic, test_constants::kPublishPeriod);

    ASSERT_FALSE(test_node_->IsStarted());

    const moving_average_statistics::StatisticData data = test_node_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_node_->Stop();
    ASSERT_FALSE(test_node_->IsStarted());
    test_node_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxProcessCpuMeasurementNode> test_node_;
};

TEST_F(LinuxProcessCpuMeasurementTestFixture, TestGetMetricName) {
  const int pid = system_metrics_collector::GetPid();
  ASSERT_EQ(std::to_string(pid) + "_cpu_percent_used", test_node_->GetMetricName());
}
