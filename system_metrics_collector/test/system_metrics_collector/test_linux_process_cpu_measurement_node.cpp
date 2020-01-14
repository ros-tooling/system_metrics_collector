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
#include <vector>

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "../../src/system_metrics_collector/constants.hpp"
#include "../../src/system_metrics_collector/linux_process_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;

namespace
{
constexpr const char kTestNodeName[] = "test_measure_linux_process_cpu";
}

class MockLinuxProcessCpuMeasurementNode : public system_metrics_collector::
  LinuxProcessCpuMeasurementNode
{
public:
  MockLinuxProcessCpuMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxProcessCpuMeasurementNode{name, options} {}

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

private:
  system_metrics_collector::ProcPidCpuData MakeSingleMeasurement() override
  {
    EXPECT_GT(test_constants::kProcPidSamples.size(), measurement_index_);
    return test_constants::kProcPidSamples[measurement_index_++];
  }

  int measurement_index_{0};
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
        std::function<void(MetricsMessage::UniquePtr)>>(
      system_metrics_collector::collector_node_constants::kStatisticsTopicName,
      10 /*history_depth*/, callback);

    // tools for calculating expected statistics values
    moving_average_statistics::MovingAverageStatistics stats_calc;
    StatisticData data;

    // setting expected_stats_[0]
    // round 1 50 ms: kProcPidSamples[0] is collected
    // round 1 80 ms: statistics derived from kProcPidSamples[N/A-0] is published
    stats_calc.Reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats_[0]);

    // setting expected_stats_[1]
    // round 1 100 ms: kProcPidSamples[1] is collected
    // round 1 150 ms: kProcPidSamples[2] is collected
    // round 1 160 ms: statistics derived from kProcPidSamples[0-1 & 1-2] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[0],
        test_constants::kProcPidSamples[1]));
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[1],
        test_constants::kProcPidSamples[2]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[1]);

    // setting expected_stats_[2]
    // round 1 200 ms: kProcPidSamples[3] is collected
    // round 1 240 ms: statistics derived from kProcPidSamples[2-3] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[2],
        test_constants::kProcPidSamples[3]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[2]);

    // setting expected_stats_[3]
    // round 2 50 ms: kProcPidSamples[5] is collected
    // round 2 80 ms: statistics derived from kProcPidSamples[N/A-5] is published
    stats_calc.Reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats_[3]);

    // setting expected_stats_[4]
    // round 2 100 ms: kProcPidSamples[6] is collected
    // round 2 150 ms: kProcPidSamples[7] is collected
    // round 2 160 ms: statistics derived from kProcPidSamples[5-6 & 6-7] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[5],
        test_constants::kProcPidSamples[6]));
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[6],
        test_constants::kProcPidSamples[7]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[4]);

    // setting expected_stats_[5]
    // round 2 200 ms: kProcPidSamples[8] is collected
    // round 2 240 ms: statistics derived from kProcPidSamples[7-8] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      system_metrics_collector::ComputePidCpuActivePercentage(
        test_constants::kProcPidSamples[7],
        test_constants::kProcPidSamples[8]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[5]);
  }

  int GetNumReceived() const
  {
    return times_received_;
  }

private:
  using ExpectedStatistics =
    std::unordered_map<decltype(StatisticDataPoint::data_type), decltype(StatisticDataPoint::data)>;

  void StatisticDataToExpectedStatistics(const StatisticData & src, ExpectedStatistics & dst)
  {
    dst[StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE] = src.average;
    dst[StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM] = src.min;
    dst[StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM] = src.max;
    dst[StatisticDataType::STATISTICS_DATA_TYPE_STDDEV] = src.standard_deviation;
    dst[StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT] = src.sample_count;
  }

  void MetricsMessageCallback(const MetricsMessage & msg) const
  {
    ASSERT_GT(expected_stats_.size(), times_received_);

    // check source names
    EXPECT_EQ(kTestNodeName, msg.measurement_source_name);
    EXPECT_EQ(expected_metric_name_, msg.metrics_source);

    // check measurements
    const ExpectedStatistics & expected_stat = expected_stats_[times_received_];
    EXPECT_EQ(expected_stat.size(), msg.statistics.size());

    for (const StatisticDataPoint & stat : msg.statistics) {
      EXPECT_GT(expected_stat.count(stat.data_type), 0);
      if (std::isnan(expected_stat.at(stat.data_type))) {
        EXPECT_TRUE(std::isnan(stat.data));
      } else {
        EXPECT_DOUBLE_EQ(expected_stat.at(stat.data_type), stat.data);
      }
    }

    ++times_received_;
  }

  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
  std::string expected_metric_name_;
  std::array<ExpectedStatistics, 6> expected_stats_;
  mutable int times_received_;
};

class LinuxProcessCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      test_constants::kMeasurePeriod.count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      test_constants::kPublishPeriod.count());

    test_node_ = std::make_shared<MockLinuxProcessCpuMeasurementNode>(
      kTestNodeName, options);

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
  std::shared_ptr<MockLinuxProcessCpuMeasurementNode> test_node_;
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
  ASSERT_DOUBLE_EQ(test_constants::kCpuActiveProcPidSample_0_1, cpu_active_percentage);
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
  ex.add_node(test_node_->get_node_base_interface());
  ex.add_node(test_receive_measurements->get_node_base_interface());

  //
  // spin the node with it started
  //
  bool start_success = test_node_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_node_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // 50 ms: kProcPidSamples[0] is collected
  // 80 ms: statistics derived from kProcPidSamples[N/A-0] is published. statistics are cleared
  // 100 ms: kProcPidSamples[1] is collected
  // 150 ms: kProcPidSamples[2] is collected
  // 160 ms: statistics derived from kProcPidSamples[0-1 & 1-2] is published. statistics are cleared
  // 200 ms: kProcPidSamples[3] is collected
  // 240 ms: statistics derived from kProcPidSamples[2-3] is published. statistics are cleared
  // 250 ms: kProcPidSamples[4] is collected. last GetStatisticsResults() is of kProcPidSamples[3-4]
  StatisticData data = test_node_->GetStatisticsResults();
  double expected_cpu_active = system_metrics_collector::ComputePidCpuActivePercentage(
    test_constants::kProcPidSamples[3], test_constants::kProcPidSamples[4]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
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
  // expectation is:
  // 50 ms: kProcPidSamples[5] is collected
  // 80 ms: statistics derived from kProcPidSamples[N/A-5] is published. statistics are cleared
  // 100 ms: kProcPidSamples[6] is collected
  // 150 ms: kProcPidSamples[7] is collected
  // 160 ms: statistics derived from kProcPidSamples[5-6 & 6-7] is published. statistics are cleared
  // 200 ms: kProcPidSamples[8] is collected
  // 240 ms: statistics derived from kProcPidSamples[7-8] is published. statistics are cleared
  // 250 ms: kProcPidSamples[9] is collected. last GetStatisticsResults() is of kProcPidSamples[8-9]
  data = test_node_->GetStatisticsResults();
  expected_cpu_active = system_metrics_collector::ComputePidCpuActivePercentage(
    test_constants::kProcPidSamples[8],
    test_constants::kProcPidSamples[9]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);
}
