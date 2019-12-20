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
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using system_metrics_collector::ProcCpuData;
using system_metrics_collector::processPidStatCpuLine;
using system_metrics_collector::processStatCpuLine;
using system_metrics_collector::ProcPidCpuData;
using test_constants::PROC_PID_SAMPLES;
using test_constants::PROC_SAMPLES;
using test_utilities::computePidCpuActivePercentage;

namespace
{
constexpr const char TEST_NODE_NAME[] = "test_measure_linux_process_cpu";
constexpr const char TEST_TOPIC[] = "test_process_cpu_measure_topic";
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
  : LinuxProcessCpuMeasurementNode(name, measurement_period, topic, publish_period),
    measurement_index(0) {}

  double periodicMeasurement() override
  {
    LinuxProcessCpuMeasurementNode::periodicMeasurement();
  }

  std::string getMetricName() const override
  {
    return LinuxProcessCpuMeasurementNode::getMetricName();
  }

private:
  std::tuple<ProcPidCpuData, ProcCpuData> makeSingleMeasurement() override
  {
    EXPECT_GT(test_constants::PROC_SAMPLES.size(), measurement_index);
    EXPECT_GT(test_constants::PROC_PID_SAMPLES.size(), measurement_index);
    auto ret_val = std::make_tuple(processPidStatCpuLine(
          PROC_PID_SAMPLES[measurement_index]),
        processStatCpuLine(PROC_SAMPLES[measurement_index]));
    ++measurement_index;
    return ret_val;
  }

  int measurement_index;
};

class TestReceiveProcessCpuMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveProcessCpuMeasurementNode(
    const std::string & name,
    const std::string & metric)
  : rclcpp::Node(name), expected_metric_name(metric), times_received(0)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(TEST_TOPIC, 10 /*history_depth*/, callback);

    // tools for calculating expected statistics values
    moving_average_statistics::MovingAverageStatistics stats_calc;
    StatisticData data;

    // setting expected_stats[0]
    // round 1 50 ms: PROC_SAMPLES[0] is collected
    // round 1 80 ms: statistics derived from PROC_SAMPLES[N/A-0] is published
    stats_calc.reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats[0]);

    // setting expected_stats[1]
    // round 1 100 ms: PROC_SAMPLES[1] is collected
    // round 1 150 ms: PROC_SAMPLES[2] is collected
    // round 1 160 ms: statistics derived from PROC_SAMPLES[0-1 & 1-2] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[0], PROC_SAMPLES[0],
      PROC_PID_SAMPLES[1], PROC_SAMPLES[1]));
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[1], PROC_SAMPLES[1],
      PROC_PID_SAMPLES[2], PROC_SAMPLES[2]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[1]);

    // setting expected_stats[2]
    // round 1 200 ms: PROC_SAMPLES[3] is collected
    // round 1 240 ms: statistics derived from PROC_SAMPLES[2-3] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[2], PROC_SAMPLES[2],
      PROC_PID_SAMPLES[3], PROC_SAMPLES[3]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[2]);

    // setting expected_stats[3]
    // round 2 50 ms: PROC_SAMPLES[5] is collected
    // round 2 80 ms: statistics derived from PROC_SAMPLES[N/A-5] is published
    stats_calc.reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats[3]);

    // setting expected_stats[4]
    // round 2 100 ms: PROC_SAMPLES[6] is collected
    // round 2 150 ms: PROC_SAMPLES[7] is collected
    // round 2 160 ms: statistics derived from PROC_SAMPLES[5-6 & 6-7] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[5], PROC_SAMPLES[5],
      PROC_PID_SAMPLES[6], PROC_SAMPLES[6]));
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[6], PROC_SAMPLES[6],
      PROC_PID_SAMPLES[7], PROC_SAMPLES[7]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[4]);

    // setting expected_stats[5]
    // round 2 200 ms: PROC_SAMPLES[8] is collected
    // round 2 240 ms: statistics derived from PROC_SAMPLES[7-8] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computePidCpuActivePercentage(PROC_PID_SAMPLES[7], PROC_SAMPLES[7],
      PROC_PID_SAMPLES[8], PROC_SAMPLES[8]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[5]);
  }

  int getNumReceived() const
  {
    return times_received;
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
    ASSERT_GT(expected_stats.size(), times_received);

    // check source names
    EXPECT_EQ(TEST_NODE_NAME, msg.measurement_source_name);
    EXPECT_EQ(expected_metric_name, msg.metrics_source);

    // check measurements
    const ExpectedStatistics & expected_stat = expected_stats[times_received];
    EXPECT_EQ(expected_stat.size(), msg.statistics.size());

    for (const StatisticDataPoint & stat : msg.statistics) {
      EXPECT_GT(expected_stat.count(stat.data_type), 0);
      if (std::isnan(expected_stat.at(stat.data_type))) {
        EXPECT_TRUE(std::isnan(stat.data));
      } else {
        EXPECT_DOUBLE_EQ(expected_stat.at(stat.data_type), stat.data);
      }
    }

    ++times_received;
  }

  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription;
  std::string expected_metric_name;
  std::array<ExpectedStatistics, 6> expected_stats;
  mutable int times_received;
};

class LinuxProcessCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    rclcpp::init(0, nullptr);

    test_node = std::make_shared<TestLinuxProcessCpuMeasurementNode>(TEST_NODE_NAME,
        test_constants::MEASURE_PERIOD, TEST_TOPIC, test_constants::PUBLISH_PERIOD);

    ASSERT_FALSE(test_node->isStarted());

    const moving_average_statistics::StatisticData data = test_node->getStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_node->stop();
    ASSERT_FALSE(test_node->isStarted());
    test_node.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxProcessCpuMeasurementNode> test_node;
};

TEST_F(LinuxProcessCpuMeasurementTestFixture, testGetMetricName) {
  const int pid = system_metrics_collector::getPid();
  ASSERT_EQ(std::to_string(pid) + "_cpu_percent_used", test_node->getMetricName());
}

TEST_F(LinuxProcessCpuMeasurementTestFixture, testManualMeasurement)
{
  // first measurement caches
  double cpu_active_percentage = test_node->periodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_node->periodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::CPU_ACTIVE_PROC_PID_SAMPLE_0_1, cpu_active_percentage);
}

TEST_F(LinuxProcessCpuMeasurementTestFixture, testPublishMetricsMessage)
{
  ASSERT_NE(test_node, nullptr);
  ASSERT_FALSE(test_node->isStarted());

  auto test_receive_measurements = std::make_shared<TestReceiveProcessCpuMeasurementNode>(
    "test_receive_measurements", test_node->getMetricName());
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_node);
  ex.add_node(test_receive_measurements);

  //
  // spin the node with it started
  //
  bool start_success = test_node->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_node->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_DURATION);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: PROC_SAMPLES[0] is collected
  // 80 ms: statistics derived from PROC_SAMPLES[N/A-0] is published. statistics are cleared
  // 100 ms: PROC_SAMPLES[1] is collected
  // 150 ms: PROC_SAMPLES[2] is collected
  // 160 ms: statistics derived from PROC_SAMPLES[0-1 & 1-2] is published. statistics are cleared
  // 200 ms: PROC_SAMPLES[3] is collected
  // 240 ms: statistics derived from PROC_SAMPLES[2-3] is published. statistics are cleared
  // 250 ms: PROC_SAMPLES[4] is collected. last getStatisticsResults() is of PROC_SAMPLES[3-4]
  StatisticData data = test_node->getStatisticsResults();
  double expected_cpu_active = computePidCpuActivePercentage(PROC_PID_SAMPLES[3], PROC_SAMPLES[3],
      PROC_PID_SAMPLES[4], PROC_SAMPLES[4]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it stopped
  //
  bool stop_success = test_node->stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_node->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_DURATION);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so getStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_node->getStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  start_success = test_node->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_node->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_DURATION);
  EXPECT_EQ(6, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: PROC_SAMPLES[5] is collected
  // 80 ms: statistics derived from PROC_SAMPLES[N/A-5] is published. statistics are cleared
  // 100 ms: PROC_SAMPLES[6] is collected
  // 150 ms: PROC_SAMPLES[7] is collected
  // 160 ms: statistics derived from PROC_SAMPLES[5-6 & 6-7] is published. statistics are cleared
  // 200 ms: PROC_SAMPLES[8] is collected
  // 240 ms: statistics derived from PROC_SAMPLES[7-8] is published. statistics are cleared
  // 250 ms: PROC_SAMPLES[9] is collected. last getStatisticsResults() is of PROC_SAMPLES[8-9]
  data = test_node->getStatisticsResults();
  expected_cpu_active = computePidCpuActivePercentage(PROC_PID_SAMPLES[8], PROC_SAMPLES[8],
      PROC_PID_SAMPLES[9], PROC_SAMPLES[9]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);
}
