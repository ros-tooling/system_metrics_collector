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

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "../../src/system_metrics_collector/constants.hpp"
#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using system_metrics_collector::ProcessMemInfoLines;

namespace
{
constexpr const char kTestNodeName[] = "test_measure_linux_memory";
constexpr const char kTestMetricName[] = "system_memory_percent_used";

constexpr const std::array<const char *, 10> kSamples = {
  test_constants::kFullSample,

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
  TestLinuxMemoryMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxMemoryMeasurementNode{name, options},
    measurement_index_(0) {}

  ~TestLinuxMemoryMeasurementNode() override = default;

  void SetTestString(const std::string & test_string)
  {
    measurement_index_ = kInvalidIndex;
    test_string_ = test_string;
  }

  // override to avoid calling methods involved in file i/o
  double PeriodicMeasurement() override
  {
    if (measurement_index_ == kInvalidIndex) {
      return ProcessMemInfoLines(test_string_);
    } else {
      EXPECT_GT(kSamples.size(), measurement_index_);
      return ProcessMemInfoLines(kSamples[measurement_index_++]);
    }
  }

private:
  static constexpr int kInvalidIndex = -1;
  int measurement_index_;
  std::string test_string_;
};

constexpr int TestLinuxMemoryMeasurementNode::kInvalidIndex;

class LinuxMemoryMeasurementTestFixture : public ::testing::Test
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

    test_measure_linux_memory_ = std::make_shared<TestLinuxMemoryMeasurementNode>(
      kTestNodeName, options);

    ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

    const moving_average_statistics::StatisticData data =
      test_measure_linux_memory_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_memory_->Stop();
    ASSERT_FALSE(test_measure_linux_memory_->IsStarted());
    test_measure_linux_memory_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxMemoryMeasurementNode> test_measure_linux_memory_;
};

class TestReceiveMemoryMeasurementNode : public rclcpp::Node
{
public:
  explicit TestReceiveMemoryMeasurementNode(const std::string & name)
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
    // round 1 50 ms: SAMPLES[0] is collected
    // round 1 80 ms: statistics derived from SAMPLES[0] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[0]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[0]);

    // setting expected_stats_[1]
    // round 1 100 ms: SAMPLES[1] is collected
    // round 1 150 ms: SAMPLES[2] is collected
    // round 1 160 ms: statistics derived from SAMPLES[1 & 2] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[1]));
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[2]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[1]);

    // setting expected_stats_[2]
    // round 1 200 ms: SAMPLES[3] is collected
    // round 1 240 ms: statistics derived from SAMPLES[3] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[3]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[2]);

    // setting expected_stats_[3]
    // round 2 50 ms: SAMPLES[5] is collected
    // round 2 80 ms: statistics derived from SAMPLES[5] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[5]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[3]);

    // setting expected_stats_[4]
    // round 2 100 ms: SAMPLES[6] is collected
    // round 2 150 ms: SAMPLES[7] is collected
    // round 2 160 ms: statistics derived from SAMPLES[6 & 7] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[6]));
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[7]));
    data = stats_calc.GetStatistics();
    StatisticDataToExpectedStatistics(data, expected_stats_[4]);

    // setting expected_stats_[5]
    // round 2 200 ms: SAMPLES[8] is collected
    // round 2 240 ms: statistics derived from SAMPLES[8] is published
    stats_calc.Reset();
    stats_calc.AddMeasurement(ProcessMemInfoLines(kSamples[8]));
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

TEST(LinuxMemoryMeasurementTest, TestReadInvalidFile)
{
  const auto s = system_metrics_collector::ReadFileToString("this_will_fail.txt");
  ASSERT_EQ("", s);
}

TEST_F(LinuxMemoryMeasurementTestFixture, testManualMeasurement) {
  test_measure_linux_memory_->SetTestString("");
  double mem_used_percentage = test_measure_linux_memory_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(mem_used_percentage));

  test_measure_linux_memory_->SetTestString(test_constants::kFullSample);
  mem_used_percentage = test_measure_linux_memory_->PeriodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, mem_used_percentage);
}

TEST_F(LinuxMemoryMeasurementTestFixture, TestPublishMetricsMessage)
{
  ASSERT_NE(test_measure_linux_memory_, nullptr);
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

  auto test_receive_measurements = std::make_shared<TestReceiveMemoryMeasurementNode>(
    "test_receive_measurements");
  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();
  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_memory_->get_node_base_interface());
  ex.add_node(test_receive_measurements->get_node_base_interface());

  //
  // spin the node with it started
  //
  bool start_success = test_measure_linux_memory_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_memory_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // 50 ms: SAMPLES[0] is collected
  // 80 ms: statistics derived from SAMPLES[0] is published. statistics are cleared
  // 100 ms: SAMPLES[1] is collected
  // 150 ms: SAMPLES[2] is collected
  // 160 ms: statistics derived from SAMPLES[1 & 2] is published. statistics are cleared
  // 200 ms: SAMPLES[3] is collected
  // 240 ms: statistics derived from SAMPLES[3] is published. statistics are cleared
  // 250 ms: SAMPLES[4] is collected. last GetStatisticsResults() is of SAMPLES[4]
  StatisticData data = test_measure_linux_memory_->GetStatisticsResults();
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it stopped
  //
  bool stop_success = test_measure_linux_memory_->Stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(3, test_receive_measurements->GetNumReceived());
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_measure_linux_memory_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  //
  // spin the node with it restarted
  //
  start_success = test_measure_linux_memory_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_measure_linux_memory_->IsStarted());
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);
  EXPECT_EQ(6, test_receive_measurements->GetNumReceived());
  // expectation is:
  // 50 ms: SAMPLES[5] is collected
  // 80 ms: statistics derived from SAMPLES[5] is published. statistics are cleared
  // 100 ms: SAMPLES[6] is collected
  // 150 ms: SAMPLES[7] is collected
  // 160 ms: statistics derived from SAMPLES[6 & 7] is published. statistics are cleared
  // 200 ms: SAMPLES[8] is collected
  // 240 ms: statistics derived from SAMPLES[8] is published. statistics are cleared
  // 250 ms: SAMPLES[9] is collected. last GetStatisticsResults() is of SAMPLES[9]
  data = test_measure_linux_memory_->GetStatisticsResults();
  EXPECT_EQ(1, data.sample_count);
}
