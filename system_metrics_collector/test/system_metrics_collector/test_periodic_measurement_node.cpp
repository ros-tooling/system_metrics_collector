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
#include <iostream>
#include <string>
#include <memory>
#include <mutex>

#include "../../src/system_metrics_collector/collector.hpp"
#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"
#include "../../src/moving_average_statistics/types.hpp"

namespace
{
static constexpr const std::chrono::milliseconds TEST_LENGTH =
  std::chrono::milliseconds(250);
static constexpr const std::chrono::milliseconds TEST_PERIOD =
  std::chrono::milliseconds(50);
}

/**
 * Simple extension to test basic functionality
 */
class TestPeriodicMeasurementNode : public PeriodicMeasurementNode
{
public:
  TestPeriodicMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic)
  : PeriodicMeasurementNode(name, measurement_period, publishing_topic)
  {}
  virtual ~TestPeriodicMeasurementNode() = default;

  // made public to manually test
  void periodicMeasurement() override
  {
    sum += 1;
    acceptData(static_cast<double>(sum.load()));
  }

private:
  std::atomic<int> sum{0};
};

/**
 * Test fixture
 */
class PeriodicMeasurementTestFixure : public ::testing::Test
{
public:
  void SetUp() override
  {
    const char * const argv = "d";
    rclcpp::init(1, &argv);

    test_periodic_measurer = std::make_shared<TestPeriodicMeasurementNode>("test_periodic_node",
        TEST_PERIOD, "test_topic");

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
  ASSERT_EQ("name=test_periodic_node, measurement_period=50ms, publishing_topic=test_topic,"
    " started=false, avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_periodic_measurer->getStatusString());
}

TEST_F(PeriodicMeasurementTestFixure, test_measurement_manually) {
  ASSERT_NE(test_periodic_measurer, nullptr);

  test_periodic_measurer->periodicMeasurement();

  StatisticData data = test_periodic_measurer->getStatisticsResults();
  ASSERT_FALSE(std::isnan(data.average));
  ASSERT_FALSE(std::isnan(data.min));
  ASSERT_FALSE(std::isnan(data.max));
  ASSERT_FALSE(std::isnan(data.standard_deviation));
  ASSERT_EQ(1, data.sample_count);
}

TEST_F(PeriodicMeasurementTestFixure, test_start_and_stop) {
  ASSERT_TRUE(true);
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
  ASSERT_EQ(3, data.average);
  ASSERT_EQ(1, data.min);
  ASSERT_EQ(TEST_LENGTH.count() / TEST_PERIOD.count(), data.max);
  ASSERT_FALSE(std::isnan(data.standard_deviation));
  ASSERT_EQ(TEST_LENGTH.count() / TEST_PERIOD.count(), data.sample_count);

  const bool stop_success = test_periodic_measurer->stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_periodic_measurer->isStarted());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
