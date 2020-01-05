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
#include <fstream>
#include <memory>
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

  /**
   * Exposes the protected member function for testing purposes.
   * See description for LinuxProcessCpuMeasurementNode::PeriodicMeasurement().
   *
   * @return percentage of CPU this process used
   */
  double PeriodicMeasurement() override
  {
    LinuxProcessCpuMeasurementNode::PeriodicMeasurement();
  }

  /**
   * Exposes the protected member function for testing purposes.
   * See description for LinuxProcessCpuMeasurementNode::GetMetricName().
   *
   * @return a string of the name for this measured metric
   */
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
  : rclcpp::Node(name), expected_metric_name_(metric), times_received_(0)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription_ = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(kTestTopic, 10 /*history_depth*/, callback);
  }

  int GetNumReceived() const
  {
    return times_received_;
  }

private:
  void MetricsMessageCallback(const MetricsMessage & msg) const
  {
    // Given kPublishPeriod is 80 ms and kTestDuration is 250 ms, the expectation is:
    // MetricsMessages are published/received at 80 ms, 160 ms, and 240 ms during the first round.
    // The TestLinuxProcessCpuMeasurementNode is then stopped and restarted and again
    // MetricsMessages are published/received at 80 ms, 160 ms, and 240 ms during the second round.
    // This means that no more than 6 MetricsMessages are received.
    ASSERT_GT(6, times_received_);

    // check source names
    EXPECT_EQ(kTestNodeName, msg.measurement_source_name);
    EXPECT_EQ(expected_metric_name_, msg.metrics_source);

    // Check measurements.
    // There are five types of statistics:
    // average, maximum, minimum, standard deviation, number of samples
    EXPECT_EQ(5, msg.statistics.size());

    for (const StatisticDataPoint & stat : msg.statistics) {
      EXPECT_GT(stat.data_type, 0);
      if (!std::isnan(stat.data)) {
        EXPECT_GE(stat.data, 0.0);
      }
    }

    ++times_received_;
  }

  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
  std::string expected_metric_name_;
  mutable int times_received_;
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

TEST_F(LinuxProcessCpuMeasurementTestFixture, TestManualMeasurement)
{
  // first measurement caches
  double cpu_active_percentage = test_node_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_node_->PeriodicMeasurement();
  ASSERT_GE(cpu_active_percentage, 0.0);
}

TEST_F(LinuxProcessCpuMeasurementTestFixture, TestPublishMetricsMessage)
{
  ASSERT_NE(test_node_, nullptr);
  ASSERT_FALSE(test_node_->IsStarted());

  auto test_receive_measurements = std::make_shared<TestReceiveProcessCpuMeasurementNode>(
    "test_receive_measurements", test_node_->GetMetricName());
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_node_);
  ex.add_node(test_receive_measurements);

  //
  // spin the node with it started
  //
  bool start_success = test_node_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_node_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(3, test_receive_measurements->GetNumReceived());

  StatisticData data = test_node_->GetStatisticsResults();
  EXPECT_GE(data.average, 0.0);
  EXPECT_GE(data.min, 0.0);
  EXPECT_GE(data.max, 0.0);
  EXPECT_GE(data.standard_deviation, 0.0);
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it stopped
  //
  bool stop_success = test_node_->Stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_node_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_node_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  start_success = test_node_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_node_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(6, test_receive_measurements->GetNumReceived());

  data = test_node_->GetStatisticsResults();
  EXPECT_GE(data.average, 0.0);
  EXPECT_GE(data.min, 0.0);
  EXPECT_GE(data.max, 0.0);
  EXPECT_GE(data.standard_deviation, 0.0);
  EXPECT_EQ(1, data.sample_count);
}
