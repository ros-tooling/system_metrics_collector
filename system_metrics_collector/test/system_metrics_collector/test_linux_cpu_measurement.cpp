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

#include "metrics_statistics_msgs/msg/metrics_message.hpp"

#include "../../src/system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using moving_average_statistics::STATISTICS_DATA_TYPES;
using system_metrics_collector::processStatCpuLine;

namespace {

constexpr const char TEST_NODE_NAME[] = "test_measure_linux_cpu";
constexpr const char TEST_TOPIC[] = "test_cpu_measure_topic";

constexpr const std::chrono::milliseconds TEST_LENGTH =
  std::chrono::milliseconds(250);
constexpr const std::chrono::milliseconds MEASURE_PERIOD =
  std::chrono::milliseconds(50);
constexpr const std::chrono::milliseconds PUBLISH_PERIOD =
  std::chrono::milliseconds(80);

constexpr const std::array<const char *, 10> proc_samples = {
  "cpu 22451232 118653 7348045 934943300 5378119 0 419114 0 0 0\n",
  "cpu 22451360 118653 7348080 934949227 5378120 0 419117 0 0 0\n",
  "cpu 24343452 61856 6484430 10645595 58695 0 683052 0 0 0\n",
  "cpu 6051294 43322 1611333 9021635 47400 0 177494 0 0 0\n",
  "cpu 6092443 6217 1623536 535731 4143 0 232286 0 0 0\n",
  "cpu 6097071 6498 1612044 544445 3484 0 135942 0 0 0\n",
  "cpu 6102643 5818 1637516 543782 3666 0 137329 0 0 0\n",
  "cpu 24513632 62372 6527524 11205004 62394 0 687176 0 0 0\n",
  "cpu 6093617 43419 1621953 9161570 48047 0 178792 0 0 0\n",
  "cpu 6134250 6371 1634411 675446 5285 0 233478 0 0 0\n"
};

double computeCpuActivePercentage(const std::string & data1, const std::string & data2)
{
  auto parsed_data1 = processStatCpuLine(data1);
  auto parsed_data2 = processStatCpuLine(data2);
  return computeCpuActivePercentage(parsed_data1, parsed_data2);
}

constexpr const double CPU_ACTIVE_PROC_SAMPLE_0_1 = 2.7239908106334099;

}  // namespace


class TestLinuxCpuMeasurementNode : public system_metrics_collector::LinuxCpuMeasurementNode
{
public:
  TestLinuxCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic,
    const std::chrono::milliseconds publish_period)
    : LinuxCpuMeasurementNode(name, measurement_period, publishing_topic, publish_period) {}

  virtual ~TestLinuxCpuMeasurementNode() = default;

  double periodicMeasurement() override
  {
    LinuxCpuMeasurementNode::periodicMeasurement();
  }

private:
  system_metrics_collector::ProcCpuData makeSingleMeasurement() override
  {
    EXPECT_GT(proc_samples.size(), measurement_index);
    return processStatCpuLine(proc_samples[measurement_index++]);
  }

  int measurement_index{0};
};

class TestReceiveCpuMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveCpuMeasurementNode(const std::string & name)
    : rclcpp::Node(name), times_received(0)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription = create_subscription<MetricsMessage,
    std::function<void(MetricsMessage::UniquePtr)>>(TEST_TOPIC, 10, callback);

    // tools for calculating expected statistics values
    moving_average_statistics::MovingAverageStatistics stats_calc;
    StatisticData data;

    // setting expected_stats[0]
    // round 1 50 ms: proc_samples[0] is collected
    // round 1 80 ms: statistics derived from proc_samples[N/A-0] is published
    stats_calc.reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats[0]);

    // setting expected_stats[1]
    // round 1 100 ms: proc_samples[1] is collected
    // round 1 150 ms: proc_samples[2] is collected
    // round 1 160 ms: statistics derived from proc_samples[0-1 & 1-2] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[0], proc_samples[1]));
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[1], proc_samples[2]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[1]);

    // setting expected_stats[2]
    // round 1 200 ms: proc_samples[3] is collected
    // round 1 240 ms: statistics derived from proc_samples[2-3] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[2], proc_samples[3]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[2]);

    // setting expected_stats[3]
    // round 2 50 ms: proc_samples[5] is collected
    // round 2 80 ms: statistics derived from proc_samples[N/A-5] is published
    stats_calc.reset();
    data = StatisticData();
    StatisticDataToExpectedStatistics(data, expected_stats[3]);

    // setting expected_stats[4]
    // round 2 100 ms: proc_samples[6] is collected
    // round 2 150 ms: proc_samples[7] is collected
    // round 2 160 ms: statistics derived from proc_samples[5-6 & 6-7] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[5], proc_samples[6]));
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[6], proc_samples[7]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[4]);

    // setting expected_stats[5]
    // round 2 200 ms: proc_samples[8] is collected
    // round 2 240 ms: statistics derived from proc_samples[7-8] is published
    stats_calc.reset();
    stats_calc.addMeasurement(computeCpuActivePercentage(proc_samples[7], proc_samples[8]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[5]);
  }

  int getNumReceived() const
  {
    return times_received;
  }

private:
  using ExpectedStatistics = std::unordered_map<decltype(StatisticDataPoint::data_type), decltype(StatisticDataPoint::data)>;

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
    EXPECT_EQ("cpu_usage", msg.metrics_source);

    // check measurement window
    // std::chrono::seconds window_sec(msg.window_stop.sec - msg.window_start.sec);
    // std::chrono::nanoseconds window_nanosec(msg.window_stop.nanosec - msg.window_start.nanosec);
    // std::chrono::milliseconds window = std::chrono::duration_cast<std::chrono::milliseconds>(
    //   window_sec + window_nanosec);
    // EXPECT_GT(5, std::abs(window.count() - PUBLISH_PERIOD.count()));

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
  std::array<ExpectedStatistics, 6> expected_stats;
  mutable int times_received;
};

class LinuxCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_measure_linux_cpu = std::make_shared<TestLinuxCpuMeasurementNode>(TEST_NODE_NAME,
                                                                           MEASURE_PERIOD, TEST_TOPIC, PUBLISH_PERIOD);

    ASSERT_FALSE(test_measure_linux_cpu->isStarted());

    const StatisticData data = test_measure_linux_cpu->getStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_cpu->stop();
    test_measure_linux_cpu.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxCpuMeasurementNode> test_measure_linux_cpu;
};

TEST(LinuxCpuMeasurementTest, testParseProcLine)
{
  auto parsed_data = processStatCpuLine(proc_samples[0]);

  ASSERT_EQ("cpu", parsed_data.cpu_label);
  ASSERT_EQ(22451232, parsed_data.times[0]);
  ASSERT_EQ(118653, parsed_data.times[1]);
  ASSERT_EQ(7348045, parsed_data.times[2]);
  ASSERT_EQ(934943300, parsed_data.times[3]);
  ASSERT_EQ(5378119, parsed_data.times[4]);
  ASSERT_EQ(0, parsed_data.times[5]);
  ASSERT_EQ(419114, parsed_data.times[6]);
  ASSERT_EQ(0, parsed_data.times[7]);

  ASSERT_EQ(
    "cpu_label=cpu, user=22451232, nice=118653, system=7348045, idle=934943300,"
    " iOWait=5378119, irq=0, softIrq=419114, steal=0",
    parsed_data.toString());
}

TEST(LinuxCpuMeasurementTest, testCalculateCpuActivePercentage)
{
  auto p = computeCpuActivePercentage(proc_samples[0], proc_samples[1]);
  ASSERT_DOUBLE_EQ(CPU_ACTIVE_PROC_SAMPLE_0_1, p);
}

TEST(LinuxCpuMeasurementTest, testEmptyProcCpuData)
{
  system_metrics_collector::ProcCpuData empty;
  ASSERT_EQ(system_metrics_collector::ProcCpuData::EMPTY_LABEL, empty.cpu_label);

  for (int i = 0; i < static_cast<int>(system_metrics_collector::ProcCpuStates::kNumProcCpuStates);
       i++) {
    ASSERT_EQ(0, empty.times[i]);
  }
}

TEST_F(LinuxCpuMeasurementTestFixture, testManualMeasurement)
{
  // first measurement caches
  double cpu_active_percentage = test_measure_linux_cpu->periodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_measure_linux_cpu->periodicMeasurement();
  ASSERT_DOUBLE_EQ(CPU_ACTIVE_PROC_SAMPLE_0_1, cpu_active_percentage);
}

TEST_F(LinuxCpuMeasurementTestFixture, testPublishMetricsMessage)
{
  ASSERT_NE(test_measure_linux_cpu, nullptr);
  ASSERT_FALSE(test_measure_linux_cpu->isStarted());

  auto test_receive_measurements = std::make_shared<TestReceiveCpuMeasurementNode>(
                                     "test_receive_measurements");
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_cpu);
  ex.add_node(test_receive_measurements);

  //
  // spin the node with it started
  //
  bool start_success = test_measure_linux_cpu->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_cpu->isStarted());
  ex.spin_until_future_complete(dummy_future, TEST_LENGTH);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: proc_samples[0] is collected
  // 80 ms: statistics derived from proc_samples[N/A-0] is published. statistics are cleared
  // 100 ms: proc_samples[1] is collected
  // 150 ms: proc_samples[2] is collected
  // 160 ms: statistics derived from proc_samples[0-1 & 1-2] is published. statistics are cleared
  // 200 ms: proc_samples[3] is collected
  // 240 ms: statistics derived from proc_samples[2-3] is published. statistics are cleared
  // 250 ms: proc_samples[4] is collected. last getStatisticsResults() is of proc_samples[3-4]
  StatisticData data = test_measure_linux_cpu->getStatisticsResults();
  double expected_cpu_active = computeCpuActivePercentage(proc_samples[3], proc_samples[4]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it stopped
  //
  bool stop_success = test_measure_linux_cpu->stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_measure_linux_cpu->isStarted());
  ex.spin_until_future_complete(dummy_future, TEST_LENGTH);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so getStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_measure_linux_cpu->getStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  start_success = test_measure_linux_cpu->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_cpu->isStarted());
  ex.spin_until_future_complete(dummy_future, TEST_LENGTH);
  EXPECT_EQ(6, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: proc_samples[5] is collected
  // 80 ms: statistics derived from proc_samples[N/A-5] is published. statistics are cleared
  // 100 ms: proc_samples[6] is collected
  // 150 ms: proc_samples[7] is collected
  // 160 ms: statistics derived from proc_samples[5-6 & 6-7] is published. statistics are cleared
  // 200 ms: proc_samples[8] is collected
  // 240 ms: statistics derived from proc_samples[7-8] is published. statistics are cleared
  // 250 ms: proc_samples[9] is collected. last getStatisticsResults() is of proc_samples[8-9]
  data = test_measure_linux_cpu->getStatisticsResults();
  expected_cpu_active = computeCpuActivePercentage(proc_samples[8], proc_samples[9]);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0, data.standard_deviation);
  EXPECT_EQ(1, data.sample_count);
}
