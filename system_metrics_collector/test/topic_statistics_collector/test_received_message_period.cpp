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

#include <chrono>
#include <string>
#include <thread>

#include "../../src/moving_average_statistics/types.hpp"
#include "../../src/topic_statistics_collector/received_message_period.hpp"


namespace
{
constexpr const std::chrono::seconds kDefaultDurationSeconds{1};
constexpr const int kDefaultMessage = 42;
constexpr const double kExpectedAverageMilliseconds = 1000.0;
constexpr const double kExpectedMinMilliseconds = 1000.0;
constexpr const double kExpectedMaxMilliseconds = 1000.0;
constexpr const double kExpectedStandardDeviation = 0.0;
}  // namespace

class TestReceivedMessagePeriod : public topic_statistics_collector::ReceivedMessagePeriod<int>
{
public:
  /**
   * Constructs a TestReceivedMessagePeriod object.
   * @param mock_time if true the mock time will be used, false then system time
   */
  explicit TestReceivedMessagePeriod(bool mock_time = false)
  : mock_time_(mock_time)
  {
    if (mock_time_) {
      fake_now_ = ReceivedMessagePeriod::GetCurrentTime();
    }
  }
  virtual ~TestReceivedMessagePeriod() = default;

  /**
   * Overridden in order to mock the clock for measurement testing.
   * @return
   */
  std::chrono::high_resolution_clock::time_point GetCurrentTime() override
  {
    if (mock_time_) {
      return fake_now_;
    } else {
      return ReceivedMessagePeriod::GetCurrentTime();
    }
  }

  /**
   * Advance time by a specified duration, in seconds.
   * @param seconds duration which to advance time
   */
  void AdvanceTime(std::chrono::seconds seconds)
  {
    fake_now_ += seconds;
  }

  bool mock_time_{false};
  std::chrono::high_resolution_clock::time_point fake_now_;
};

TEST(ReceivedMessagePeriodTest, GetCurrentTime) {
  TestReceivedMessagePeriod test{};

  auto now = test.GetCurrentTime();
  EXPECT_NE(topic_statistics_collector::kDefaultTimePoint, now);
}

TEST(ReceivedMessagePeriodTest, TestPeriodMeasurement) {
  TestReceivedMessagePeriod test{true};

  EXPECT_TRUE(test.IsStarted()) << "Expected to be started after constructed";

  test.OnMessageReceived(kDefaultMessage);
  auto stats = test.GetStatisticsResults();
  EXPECT_EQ(0, stats.sample_count) << "Expected 0 samples to be collected";

  test.AdvanceTime(kDefaultDurationSeconds);
  test.OnMessageReceived(kDefaultMessage);
  stats = test.GetStatisticsResults();
  EXPECT_EQ(1, stats.sample_count) << "Expected 1 sample to be collected";

  test.AdvanceTime(kDefaultDurationSeconds);
  test.OnMessageReceived(kDefaultMessage);
  stats = test.GetStatisticsResults();
  EXPECT_EQ(2, stats.sample_count) << "Expected 2 samples to be collected";

  test.AdvanceTime(kDefaultDurationSeconds);
  test.OnMessageReceived(kDefaultMessage);
  stats = test.GetStatisticsResults();
  EXPECT_EQ(3, stats.sample_count);
  EXPECT_EQ(kExpectedAverageMilliseconds, stats.average);
  EXPECT_EQ(kExpectedMinMilliseconds, stats.min);
  EXPECT_EQ(kExpectedMaxMilliseconds, stats.max);
  EXPECT_EQ(kExpectedStandardDeviation, stats.standard_deviation);
}
