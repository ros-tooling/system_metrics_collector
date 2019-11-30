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

#include <atomic>
#include <chrono>
#include <iostream>
#include <string>
#include <memory>
#include <mutex>

#include "../../src/system_metrics_collector/collector.hpp"
#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/moving_average_statistics/types.hpp"

namespace
{
constexpr const std::chrono::milliseconds TEST_LENGTH =
  std::chrono::milliseconds(250);
constexpr const std::chrono::milliseconds MEASURE_PERIOD =
  std::chrono::milliseconds(50);
constexpr const std::chrono::milliseconds PUBLISH_PERIOD =
  std::chrono::milliseconds(80);
}  // namespace

/**
 * Simple extension to test basic functionality
 */
class TestPeriodicMeasurementNode : public PeriodicMeasurementNode
{
public:
  TestPeriodicMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic,
    const std::chrono::milliseconds publish_period,
    const bool clear_measurements_on_publish)
  : PeriodicMeasurementNode(name, measurement_period, publishing_topic, publish_period,
      clear_measurements_on_publish)
  {}
  virtual ~TestPeriodicMeasurementNode() = default;

  int getNumPublished() const
  {
    return times_published;
  }

private:
  /**
   * Test measurement for the fixture.
   *
   * @return
   */
  double periodicMeasurement() override
  {
    ++times_measured;
    return static_cast<double>(times_measured.load());
  }

  /**
   * Test publish for the fixture.
   *
   * @return
   */
  void publishStatistics() override
  {
    ++times_published;
  }

  std::atomic<int> times_measured{0};
  std::atomic<int> times_published{0};
};

/**
 * Test fixture
 */
class PeriodicMeasurementTestFixure : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_periodic_measurer = std::make_shared<TestPeriodicMeasurementNode>("test_periodic_node",
        MEASURE_PERIOD, "test_topic", PUBLISH_PERIOD, false);

    ASSERT_FALSE(test_periodic_measurer->isStarted());

    const StatisticData data = test_periodic_measurer->getStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_periodic_measurer->stop();
    test_periodic_measurer.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestPeriodicMeasurementNode> test_periodic_measurer;
};

TEST_F(PeriodicMeasurementTestFixure, sanity) {
  ASSERT_NE(test_periodic_measurer, nullptr);
  ASSERT_EQ("name=test_periodic_node, measurement_period=50ms,"
    " publishing_topic=test_topic, publish_period=80ms,"
    " clear_measurements_on_publish_=0, started=false,"
    " avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_periodic_measurer->getStatusString());
}

TEST_F(PeriodicMeasurementTestFixure, test_start_and_stop) {
  ASSERT_NE(test_periodic_measurer, nullptr);
  ASSERT_FALSE(test_periodic_measurer->isStarted());

  const bool start_success = test_periodic_measurer->start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_periodic_measurer->isStarted());

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_periodic_measurer);
  ex.spin_until_future_complete(dummy_future, TEST_LENGTH);

  StatisticData data = test_periodic_measurer->getStatisticsResults();
  EXPECT_EQ(3, data.average);
  EXPECT_EQ(1, data.min);
  EXPECT_EQ(TEST_LENGTH.count() / MEASURE_PERIOD.count(), data.max);
  EXPECT_FALSE(std::isnan(data.standard_deviation));
  EXPECT_EQ(TEST_LENGTH.count() / MEASURE_PERIOD.count(), data.sample_count);

  const bool stop_success = test_periodic_measurer->stop();
  EXPECT_TRUE(stop_success);
  EXPECT_FALSE(test_periodic_measurer->isStarted());

  int times_published = test_periodic_measurer->getNumPublished();
  EXPECT_EQ(TEST_LENGTH.count() / PUBLISH_PERIOD.count(), times_published);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
