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

#include <array>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "system_metrics_collector/proc_cpu_data.hpp"
#include "system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

#include "rclcpp/rclcpp.hpp"

using lifecycle_msgs::msg::State;
using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using system_metrics_collector::ProcessStatCpuLine;
using test_constants::kProcSamples;

namespace
{
constexpr const char kTestNodeName[] = "test_measure_linux_cpu";
constexpr const char kTestMetricName[] = "system_cpu_percent_used";
}  // namespace

/**
 * Test class used to fake out linux CPU measurements
 */
class TestLinuxCpuMeasurementNode : public system_metrics_collector::LinuxCpuMeasurementNode
{
public:
  TestLinuxCpuMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxCpuMeasurementNode{name, options} {}

  ~TestLinuxCpuMeasurementNode() override = default;

  // make this private method public for unit testing purposes
  double PeriodicMeasurement() override
  {
    LinuxCpuMeasurementNode::PeriodicMeasurement();
  }

private:
  system_metrics_collector::ProcCpuData MakeSingleMeasurement() override
  {
    EXPECT_GT(kProcSamples.size(), measurement_index_);
    return ProcessStatCpuLine(kProcSamples[measurement_index_++]);
  }

  int measurement_index_{0};
};
/**
 * Subscriber for messages sent by the TestLinuxCpuMeasurementNode
 */
class TestReceiveCpuMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveCpuMeasurementNode(const std::string & name)
  : rclcpp::Node(name), times_received_(0)
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
    // round 1 50 ms: kProcSamples[0] is collected
    // round 1 80 ms: statistics derived from kProcSamples[N/A-0] is published
    stats_calc.Reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats_[0]);

    // setting expected_stats_[1]
    // round 1 100 ms: kProcSamples[1] is collected
    // round 1 150 ms: kProcSamples[2] is collected
    // round 1 160 ms: statistics derived from kProcSamples[0-1 & 1-2] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[0]),
        ProcessStatCpuLine(kProcSamples[1])));
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[1]),
        ProcessStatCpuLine(kProcSamples[2])));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[1]);

    // setting expected_stats_[2]
    // round 1 200 ms: kProcSamples[3] is collected
    // round 1 240 ms: statistics derived from kProcSamples[2-3] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[2]),
        ProcessStatCpuLine(kProcSamples[3])));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[2]);

    // setting expected_stats_[3]
    // round 2 50 ms: kProcSamples[5] is collected
    // round 2 80 ms: statistics derived from kProcSamples[N/A-5] is published
    stats_calc.Reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats_[3]);

    // setting expected_stats_[4]
    // round 2 100 ms: kProcSamples[6] is collected
    // round 2 150 ms: kProcSamples[7] is collected
    // round 2 160 ms: statistics derived from kProcSamples[5-6 & 6-7] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[5]),
        ProcessStatCpuLine(kProcSamples[6])));
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[6]),
        ProcessStatCpuLine(kProcSamples[7])));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[4]);

    // setting expected_stats_[5]
    // round 2 200 ms: kProcSamples[8] is collected
    // round 2 240 ms: statistics derived from kProcSamples[7-8] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(
      ComputeCpuActivePercentage(
        ProcessStatCpuLine(kProcSamples[7]),
        ProcessStatCpuLine(kProcSamples[8])));
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
    EXPECT_EQ(kTestMetricName, msg.metrics_source);
    EXPECT_EQ(system_metrics_collector::collector_node_constants::kPercentUnitName, msg.unit);

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
  std::array<ExpectedStatistics, 6> expected_stats_;
  mutable int times_received_;
};

class LinuxCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      test_constants::kMeasurePeriod.count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      test_constants::kPublishPeriod.count());

    test_measure_linux_cpu_ = std::make_shared<TestLinuxCpuMeasurementNode>(
      kTestNodeName, options);

    ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

    const StatisticData data = test_measure_linux_cpu_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_cpu_->shutdown();
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_measure_linux_cpu_->get_current_state().id());
    EXPECT_FALSE(test_measure_linux_cpu_->IsStarted());

    test_measure_linux_cpu_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxCpuMeasurementNode> test_measure_linux_cpu_;
};

TEST_F(LinuxCpuMeasurementTestFixture, TestManualMeasurement)
{
  // first measurement caches
  double cpu_active_percentage = test_measure_linux_cpu_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_measure_linux_cpu_->PeriodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::kCpuActiveProcSample_0_1, cpu_active_percentage);
}

TEST_F(LinuxCpuMeasurementTestFixture, TestPublishMetricsMessage)
{
  ASSERT_NE(test_measure_linux_cpu_, nullptr);
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());
  ASSERT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_measure_linux_cpu_->get_current_state().id());

  auto test_receive_measurements = std::make_shared<TestReceiveCpuMeasurementNode>(
    "test_receive_measurements");
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;

  ex.add_node(test_measure_linux_cpu_->get_node_base_interface());
  ex.add_node(test_receive_measurements->get_node_base_interface());

  //
  // spin the node with it started
  //
  test_measure_linux_cpu_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_cpu_->get_current_state().id());

  test_measure_linux_cpu_->activate();
  ASSERT_TRUE(test_measure_linux_cpu_->IsStarted());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_cpu_->get_current_state().id());

  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);

  ASSERT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // 50 ms: kProcSamples[0] is collected
  // 80 ms: statistics derived from kProcSamples[N/A-0] is published. statistics are cleared
  // 100 ms: kProcSamples[1] is collected
  // 150 ms: kProcSamples[2] is collected
  // 160 ms: statistics derived from kProcSamples[0-1 & 1-2] is published. statistics are cleared
  // 200 ms: kProcSamples[3] is collected
  // 240 ms: statistics derived from kProcSamples[2-3] is published. statistics are cleared
  // 250 ms: kProcSamples[4] is collected. last GetStatisticsResults() is of kProcSamples[3-4]
  StatisticData data = test_measure_linux_cpu_->GetStatisticsResults();
  double expected_cpu_active = ComputeCpuActivePercentage(
    ProcessStatCpuLine(kProcSamples[3]),
    ProcessStatCpuLine(kProcSamples[4]));
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it deactivated
  //
  test_measure_linux_cpu_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  ASSERT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_measure_linux_cpu_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  test_measure_linux_cpu_->activate();
  ASSERT_TRUE(test_measure_linux_cpu_->IsStarted());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_cpu_->get_current_state().id());

  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(6, test_receive_measurements->GetNumReceived());
  // expectation is:
  // 50 ms: kProcSamples[5] is collected
  // 80 ms: statistics derived from kProcSamples[N/A-5] is published. statistics are cleared
  // 100 ms: kProcSamples[6] is collected
  // 150 ms: kProcSamples[7] is collected
  // 160 ms: statistics derived from kProcSamples[5-6 & 6-7] is published. statistics are cleared
  // 200 ms: kProcSamples[8] is collected
  // 240 ms: statistics derived from kProcSamples[7-8] is published. statistics are cleared
  // 250 ms: kProcSamples[9] is collected. last GetStatisticsResults() is of kProcSamples[8-9]
  data = test_measure_linux_cpu_->GetStatisticsResults();
  expected_cpu_active = ComputeCpuActivePercentage(
    ProcessStatCpuLine(kProcSamples[8]),
    ProcessStatCpuLine(kProcSamples[9]));
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);
}
