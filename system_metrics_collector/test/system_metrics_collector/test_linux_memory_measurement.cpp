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
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using system_metrics_collector::processMemInfoLines;

namespace
{
constexpr const char TEST_NODE_NAME[] = "test_measure_linux_memory";
constexpr const char TEST_TOPIC[] = "test_memory_measure_topic";

constexpr const std::array<const char *, 10> SAMPLES = {
  test_constants::FULL_SAMPLE,

  "MemTotal:       16304208 kB\n"
  "MemFree:          845168 kB\n"
  "MemAvailable:    4840176 kB\n",

  "MemTotal:       16302048 kB\n"
  "MemFree:         9104952 kB\n"
  "MemAvailable:     239124 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          821256 kB\n"
  "MemAvailable:    4828452 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          825460 kB\n"
  "MemAvailable:    4835920 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          826912 kB\n"
  "MemAvailable:    4837388 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          827568 kB\n"
  "MemAvailable:    4838060 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          826792 kB\n"
  "MemAvailable:    4837376 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          827380 kB\n"
  "MemAvailable:    4838020 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          826968 kB\n"
  "MemAvailable:    4837664 kB\n",
};

}  // namespace


class TestLinuxMemoryMeasurementNode : public system_metrics_collector::LinuxMemoryMeasurementNode
{
public:
  TestLinuxMemoryMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic,
    const std::chrono::milliseconds publish_period)
  : LinuxMemoryMeasurementNode(name, measurement_period, publishing_topic, publish_period),
    measurement_index(0) {}

  virtual ~TestLinuxMemoryMeasurementNode() = default;

  void setTestString(const std::string & test_string)
  {
    measurement_index = -1;
    test_string_ = test_string;
  }

  // override to avoid calling methods involved in file i/o
  double periodicMeasurement() override
  {
    if (measurement_index < 0) {
      return processMemInfoLines(test_string_);
    } else {
      EXPECT_GT(SAMPLES.size(), measurement_index);
      return processMemInfoLines(SAMPLES[measurement_index++]);
    }
  }

private:
  int measurement_index;
  std::string test_string_;
};

class LinuxMemoryMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_measure_linux_memory = std::make_shared<TestLinuxMemoryMeasurementNode>(TEST_NODE_NAME,
        test_constants::MEASURE_PERIOD, TEST_TOPIC, test_constants::PUBLISH_PERIOD);

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

class TestReceiveMemoryMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveMemoryMeasurementNode(const std::string & name)
  : rclcpp::Node(name), times_received(0)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(TEST_TOPIC, 10, callback);

    // tools for calculating expected statistics values
    moving_average_statistics::MovingAverageStatistics stats_calc;
    StatisticData data;

    // setting expected_stats[0]
    // round 1 50 ms: SAMPLES[0] is collected
    // round 1 80 ms: statistics derived from SAMPLES[0] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[0]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[0]);

    // setting expected_stats[1]
    // round 1 100 ms: SAMPLES[1] is collected
    // round 1 150 ms: SAMPLES[2] is collected
    // round 1 160 ms: statistics derived from SAMPLES[1 & 2] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[1]));
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[2]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[1]);

    // setting expected_stats[2]
    // round 1 200 ms: SAMPLES[3] is collected
    // round 1 240 ms: statistics derived from SAMPLES[3] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[3]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[2]);

    // setting expected_stats[3]
    // round 2 50 ms: SAMPLES[5] is collected
    // round 2 80 ms: statistics derived from SAMPLES[5] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[5]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[3]);

    // setting expected_stats[4]
    // round 2 100 ms: SAMPLES[6] is collected
    // round 2 150 ms: SAMPLES[7] is collected
    // round 2 160 ms: statistics derived from SAMPLES[6 & 7] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[6]));
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[7]));
    data = stats_calc.getStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats[4]);

    // setting expected_stats[5]
    // round 2 200 ms: SAMPLES[8] is collected
    // round 2 240 ms: statistics derived from SAMPLES[8] is published
    stats_calc.reset();
    stats_calc.addMeasurement(processMemInfoLines(SAMPLES[8]));
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
    EXPECT_EQ("system_memory_percent_used", msg.metrics_source);

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

TEST(LinuxMemoryMeasurementTest, testReadInvalidFile)
{
  const auto s = system_metrics_collector::readFileToString("this_will_fail.txt");
  ASSERT_EQ("", s);
}

TEST_F(LinuxMemoryMeasurementTestFixture, testManualMeasurement) {
  test_measure_linux_memory->setTestString("");
  double mem_used_percentage = test_measure_linux_memory->periodicMeasurement();
  ASSERT_TRUE(std::isnan(mem_used_percentage));

  test_measure_linux_memory->setTestString(test_constants::FULL_SAMPLE);
  mem_used_percentage = test_measure_linux_memory->periodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::MEMORY_USED_PERCENTAGE, mem_used_percentage);
}

TEST_F(LinuxMemoryMeasurementTestFixture, testPublishMetricsMessage)
{
  ASSERT_NE(test_measure_linux_memory, nullptr);
  ASSERT_FALSE(test_measure_linux_memory->isStarted());

  auto test_receive_measurements = std::make_shared<TestReceiveMemoryMeasurementNode>(
    "test_receive_measurements");
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_memory);
  ex.add_node(test_receive_measurements);

  //
  // spin the node with it started
  //
  bool start_success = test_measure_linux_memory->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_memory->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_LENGTH);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: SAMPLES[0] is collected
  // 80 ms: statistics derived from SAMPLES[0] is published. statistics are cleared
  // 100 ms: SAMPLES[1] is collected
  // 150 ms: SAMPLES[2] is collected
  // 160 ms: statistics derived from SAMPLES[1 & 2] is published. statistics are cleared
  // 200 ms: SAMPLES[3] is collected
  // 240 ms: statistics derived from SAMPLES[3] is published. statistics are cleared
  // 250 ms: SAMPLES[4] is collected. last getStatisticsResults() is of SAMPLES[4]
  StatisticData data = test_measure_linux_memory->getStatisticsResults();
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it stopped
  //
  bool stop_success = test_measure_linux_memory->stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_measure_linux_memory->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_LENGTH);
  EXPECT_EQ(3, test_receive_measurements->getNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so getStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_measure_linux_memory->getStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  start_success = test_measure_linux_memory->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_memory->isStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::TEST_LENGTH);
  EXPECT_EQ(6, test_receive_measurements->getNumReceived());
  // expectation is:
  // 50 ms: SAMPLES[5] is collected
  // 80 ms: statistics derived from SAMPLES[5] is published. statistics are cleared
  // 100 ms: SAMPLES[6] is collected
  // 150 ms: SAMPLES[7] is collected
  // 160 ms: statistics derived from SAMPLES[6 & 7] is published. statistics are cleared
  // 200 ms: SAMPLES[8] is collected
  // 240 ms: statistics derived from SAMPLES[8] is published. statistics are cleared
  // 250 ms: SAMPLES[9] is collected. last getStatisticsResults() is of SAMPLES[9]
  data = test_measure_linux_memory->getStatisticsResults();
  EXPECT_EQ(1, data.sample_count);
}
