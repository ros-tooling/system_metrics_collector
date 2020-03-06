// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <string>

#include "topic_statistics_collector/received_message_age.hpp"

#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"


namespace
{
constexpr const std::chrono::seconds kDefaultDurationSeconds{1};
constexpr const double kExpectedAverageMilliseconds{2000.0};
constexpr const double kExpectedMinMilliseconds{1000.0};
constexpr const double kExpectedMaxMilliseconds{3000.0};
constexpr const double kExpectedStandardDeviation{816.49658092772597};
const rclcpp::Time kDefaultSystemTime{0, 0, RCL_SYSTEM_TIME};
constexpr const int kDefaultTimesToTest{10};
}  // namespace

class TestReceivedMessageAgeCollector
  : public topic_statistics_collector::ReceivedMessageAgeCollector<
    sensor_msgs::msg::Imu>
{
public:
  TestReceivedMessageAgeCollector()
  {
    fake_now_ = ReceivedMessageAgeCollector::GetCurrentTime().nanoseconds();
  }
  virtual ~TestReceivedMessageAgeCollector() = default;

  /**
   * Overridden in order to mock the clock for measurement testing.
   * @return a fixed time point
   */
  rclcpp::Time GetCurrentTime() override
  {
    return rclcpp::Time{fake_now_, RCL_SYSTEM_TIME};
  }

  /**
   * Advance time by a specified duration, in seconds.
   * @param seconds duration which to advance time
   */
  void AdvanceTime(std::chrono::nanoseconds nanos)
  {
    fake_now_ += nanos.count();
  }

  int64_t fake_now_;
};

sensor_msgs::msg::Imu GetImuMessageWithHeader(const int64_t timestamp)
{
  auto message = sensor_msgs::msg::Imu{};
  message.header = std_msgs::msg::Header{};
  message.header.stamp = rclcpp::Time{timestamp};
  return message;
}

std_msgs::msg::String GetStringMessageWithoutHeader()
{
  auto message = std_msgs::msg::String{};
  message.data = "Any message with no header";
  return message;
}

TEST(ReceivedMessageAgeTest, TestOnlyMessagesWithHeaderGetSampled) {
  rclcpp::Clock clock{RCL_SYSTEM_TIME};
  topic_statistics_collector::ReceivedMessageAgeCollector<std_msgs::msg::String>
  string_msg_collector{clock};

  const auto string_msg = GetStringMessageWithoutHeader();
  moving_average_statistics::StatisticData stats;

  for (int i = 0; i < kDefaultTimesToTest; ++i) {
    string_msg_collector.OnMessageReceived(string_msg);
    stats = string_msg_collector.GetStatisticsResults();
    EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";
  }

  topic_statistics_collector::ReceivedMessageAgeCollector<sensor_msgs::msg::Imu>
  imu_msg_collector{clock};
  const auto imu_msg = GetImuMessageWithHeader(clock.now().nanoseconds());

  for (int i = 0; i < kDefaultTimesToTest; ++i) {
    imu_msg_collector.OnMessageReceived(imu_msg);
    stats = imu_msg_collector.GetStatisticsResults();
    EXPECT_EQ(i + 1, stats.sample_count) << "Expect " << i + 1 << " samples to be collected";
  }
}

TEST(ReceivedMessageAgeTest, TestMeasurementOnlyMadeForInitializedHeaderValue) {
  rclcpp::Clock clock{RCL_SYSTEM_TIME};
  topic_statistics_collector::ReceivedMessageAgeCollector<sensor_msgs::msg::Imu>
  imu_msg_collector{clock};

  // Don't initialize `header.stamp`
  const auto imu_msg_uninitialized_header = sensor_msgs::msg::Imu{};
  imu_msg_collector.OnMessageReceived(imu_msg_uninitialized_header);
  auto stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";

  // Set `header.stamp` to 0
  const auto imu_msg_zero_header = GetImuMessageWithHeader(0);
  imu_msg_collector.OnMessageReceived(imu_msg_zero_header);
  stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";

  // Set `header.stamp` to non-zero value
  const auto imu_msg_positive_header = GetImuMessageWithHeader(1);
  imu_msg_collector.OnMessageReceived(imu_msg_positive_header);
  stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(1, stats.sample_count) << "Expect 1 sample to be collected";
}

TEST(ReceivedMessageAgeTest, TestDifferentClockSourcesAreHandledCorrectly) {
  rclcpp::Clock clock{RCL_SYSTEM_TIME};

  // Default clock source is RCL_STEADY_TIME
  topic_statistics_collector::ReceivedMessageAgeCollector<sensor_msgs::msg::Imu>
  imu_msg_collector;

  const auto imu_msg = GetImuMessageWithHeader(clock.now().nanoseconds());

  imu_msg_collector.OnMessageReceived(imu_msg);
  auto stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";
}

TEST(ReceivedMessageAgeTest, TestAgeMeasurement) {
  TestReceivedMessageAgeCollector test_collector{};
  EXPECT_NE(kDefaultSystemTime, test_collector.GetCurrentTime());

  EXPECT_FALSE(test_collector.IsStarted()) << "Expect to be not started after constructed";

  EXPECT_TRUE(test_collector.Start()) << "Expect Start() to be successful";
  EXPECT_TRUE(test_collector.IsStarted()) << "Expect to be started";

  const auto test_message = GetImuMessageWithHeader(test_collector.fake_now_);

  test_collector.AdvanceTime(kDefaultDurationSeconds);
  test_collector.OnMessageReceived(test_message);
  auto stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(1, stats.sample_count);

  test_collector.AdvanceTime(kDefaultDurationSeconds);
  test_collector.OnMessageReceived(test_message);
  stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(2, stats.sample_count);

  test_collector.AdvanceTime(kDefaultDurationSeconds);
  test_collector.OnMessageReceived(test_message);
  stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(3, stats.sample_count);

  EXPECT_EQ(kExpectedAverageMilliseconds, stats.average);
  EXPECT_EQ(kExpectedMinMilliseconds, stats.min);
  EXPECT_EQ(kExpectedMaxMilliseconds, stats.max);
  EXPECT_DOUBLE_EQ(kExpectedStandardDeviation, stats.standard_deviation);
}
