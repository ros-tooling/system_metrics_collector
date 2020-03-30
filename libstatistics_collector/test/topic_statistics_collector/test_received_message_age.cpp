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

#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/received_message_age.hpp"

#include "rcl/time.h"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
using ReceivedImuMessageAgeCollector = libstatistics_collector::
  topic_statistics_collector::ReceivedMessageAgeCollector<sensor_msgs::msg::Imu>;
using ReceivedStringMessageAgeCollector = libstatistics_collector::
  topic_statistics_collector::ReceivedMessageAgeCollector<std_msgs::msg::String>;

constexpr const std::chrono::seconds kDefaultDurationSeconds{1};
constexpr const double kExpectedAverageMilliseconds{2000.0};
constexpr const double kExpectedMinMilliseconds{1000.0};
constexpr const double kExpectedMaxMilliseconds{3000.0};
constexpr const double kExpectedStandardDeviation{816.49658092772597};
constexpr const int kDefaultTimesToTest{10};
constexpr const int64_t kDefaultTimeMessageReceived{1000};
constexpr const rcl_time_point_value_t kStartTime{123456789};

sensor_msgs::msg::Imu GetImuMessageWithHeader(const int64_t seconds, const int64_t nanoseconds)
{
  auto message = sensor_msgs::msg::Imu{};
  message.header = std_msgs::msg::Header{};
  message.header.stamp.sec = seconds;
  message.header.stamp.nanosec = nanoseconds;
  return message;
}

std_msgs::msg::String GetStringMessageWithoutHeader()
{
  auto message = std_msgs::msg::String{};
  message.data = "Any message with no header";
  return message;
}
}  // namespace


TEST(ReceivedMessageAgeTest, TestOnlyMessagesWithHeaderGetSampled) {
  ReceivedStringMessageAgeCollector string_msg_collector{};

  const auto string_msg = GetStringMessageWithoutHeader();
  libstatistics_collector::moving_average_statistics::StatisticData stats;

  for (int i = 0; i < kDefaultTimesToTest; ++i) {
    string_msg_collector.OnMessageReceived(string_msg, kDefaultTimeMessageReceived);
    stats = string_msg_collector.GetStatisticsResults();
    EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";
  }

  ReceivedImuMessageAgeCollector imu_msg_collector{};
  const auto imu_msg = GetImuMessageWithHeader(1, 0);

  for (int i = 0; i < kDefaultTimesToTest; ++i) {
    imu_msg_collector.OnMessageReceived(imu_msg, kDefaultTimeMessageReceived);
    stats = imu_msg_collector.GetStatisticsResults();
    EXPECT_EQ(i + 1, stats.sample_count) << "Expect " << i + 1 << " samples to be collected";
  }
}

TEST(ReceivedMessageAgeTest, TestMeasurementOnlyMadeForInitializedHeaderValue) {
  ReceivedImuMessageAgeCollector imu_msg_collector{};

  // Don't initialize `header.stamp`
  const auto imu_msg_uninitialized_header = sensor_msgs::msg::Imu{};
  imu_msg_collector.OnMessageReceived(imu_msg_uninitialized_header, kDefaultTimeMessageReceived);
  auto stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";

  // Set `header.stamp` to 0
  const auto imu_msg_zero_header = GetImuMessageWithHeader(0, 0);
  imu_msg_collector.OnMessageReceived(imu_msg_zero_header, kDefaultTimeMessageReceived);
  stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expect 0 samples to be collected";

  // Set `header.stamp` to non-zero value
  const auto imu_msg_positive_header = GetImuMessageWithHeader(1, 0);
  imu_msg_collector.OnMessageReceived(imu_msg_positive_header, kDefaultTimeMessageReceived);
  stats = imu_msg_collector.GetStatisticsResults();
  EXPECT_EQ(1, stats.sample_count) << "Expect 1 sample to be collected";
}

TEST(ReceivedMessageAgeTest, TestAgeMeasurement) {
  ReceivedImuMessageAgeCollector test_collector{};

  EXPECT_FALSE(test_collector.IsStarted()) << "Expect to be not started after constructed";

  EXPECT_TRUE(test_collector.Start()) << "Expect Start() to be successful";
  EXPECT_TRUE(test_collector.IsStarted()) << "Expect to be started";

  rcl_time_point_value_t fake_now_nanos_{kStartTime};
  const auto test_message = GetImuMessageWithHeader(0, fake_now_nanos_);
  fake_now_nanos_ +=
    std::chrono::duration_cast<std::chrono::nanoseconds>(kDefaultDurationSeconds).count();

  test_collector.OnMessageReceived(test_message, fake_now_nanos_);
  auto stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(1, stats.sample_count);

  fake_now_nanos_ +=
    std::chrono::duration_cast<std::chrono::nanoseconds>(kDefaultDurationSeconds).count();

  test_collector.OnMessageReceived(test_message, fake_now_nanos_);
  stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(2, stats.sample_count);

  fake_now_nanos_ +=
    std::chrono::duration_cast<std::chrono::nanoseconds>(kDefaultDurationSeconds).count();

  test_collector.OnMessageReceived(test_message, fake_now_nanos_);
  stats = test_collector.GetStatisticsResults();
  EXPECT_EQ(3, stats.sample_count);

  EXPECT_EQ(kExpectedAverageMilliseconds, stats.average);
  EXPECT_EQ(kExpectedMinMilliseconds, stats.min);
  EXPECT_EQ(kExpectedMaxMilliseconds, stats.max);
  EXPECT_DOUBLE_EQ(kExpectedStandardDeviation, stats.standard_deviation);
}

TEST(ReceivedMessageAgeTest, TestGetStatNameAndUnit) {
  ReceivedImuMessageAgeCollector test_collector{};

  EXPECT_FALSE(test_collector.GetMetricName().empty());
  EXPECT_FALSE(test_collector.GetMetricUnit().empty());
}
