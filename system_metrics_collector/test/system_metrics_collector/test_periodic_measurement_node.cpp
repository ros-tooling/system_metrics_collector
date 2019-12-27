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

#include "test_constants.hpp"

namespace
{
constexpr const char kTestNodeName[] = "test_periodic_node";
constexpr const char kTestTopic[] = "test_topic";
constexpr const char kTestMetricname[] = "test_metric_name";
}  // namespace

/**
 * Simple extension to test basic functionality
 */
class TestPeriodicMeasurementNode : public ::system_metrics_collector::PeriodicMeasurementNode
{
public:
  TestPeriodicMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic,
    const std::chrono::milliseconds publish_period)
  : PeriodicMeasurementNode(name, measurement_period, publishing_topic, publish_period) {}
  virtual ~TestPeriodicMeasurementNode() = default;

  int GetNumPublished() const
  {
    return times_published_;
  }

private:
  /**
   * Test measurement for the fixture.
   *
   * @return
   */
  double PeriodicMeasurement() override
  {
    ++times_measured_;
    return static_cast<double>(times_measured_.load());
  }

  /**
   * Test publish for the fixture.
   *
   * @return
   */
  void PublishStatisticMessage() override
  {
    ++times_published_;
  }

  std::string GetMetricName() const
  {
    return kTestMetricname;
  }

  std::atomic<int> times_measured_{0};
  std::atomic<int> times_published_{0};
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

    test_periodic_measurer_ = std::make_shared<TestPeriodicMeasurementNode>(kTestNodeName,
        test_constants::kMeasurePeriod, kTestTopic, kDontPublishDuringTest);

    ASSERT_FALSE(test_periodic_measurer_->IsStarted());

    const moving_average_statistics::StatisticData data =
      test_periodic_measurer_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_periodic_measurer_->Stop();
    test_periodic_measurer_.reset();
    rclcpp::shutdown();
  }

protected:
  // this test is not designed to have the statistics published and reset at any point of the test,
  // so this is defining the publish period to be something amply larger than the test duration
  // itself
  static constexpr std::chrono::milliseconds kDontPublishDuringTest = 2 *
    test_constants::kTestDuration;
  std::shared_ptr<TestPeriodicMeasurementNode> test_periodic_measurer_;
};

constexpr std::chrono::milliseconds PeriodicMeasurementTestFixure::kDontPublishDuringTest;

TEST_F(PeriodicMeasurementTestFixure, Sanity) {
  ASSERT_NE(test_periodic_measurer_, nullptr);
  ASSERT_EQ("name=test_periodic_node, measurement_period=50ms,"
    " publishing_topic=test_topic, publish_period=500ms, started=false,"
    " avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_periodic_measurer_->GetStatusString());
}

TEST_F(PeriodicMeasurementTestFixure, TestStartAndStop) {
  ASSERT_NE(test_periodic_measurer_, nullptr);
  ASSERT_FALSE(test_periodic_measurer_->IsStarted());

  const bool start_success = test_periodic_measurer_->Start();
  ASSERT_TRUE(start_success);
  ASSERT_TRUE(test_periodic_measurer_->IsStarted());

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_periodic_measurer_);
  ex.spin_until_future_complete(dummy_future, test_constants::kTestDuration);

  moving_average_statistics::StatisticData data = test_periodic_measurer_->GetStatisticsResults();
  ASSERT_EQ(3, data.average);
  ASSERT_EQ(1, data.min);
  ASSERT_EQ(test_constants::kTestDuration.count() / test_constants::kMeasurePeriod.count(),
    data.max);
  ASSERT_FALSE(std::isnan(data.standard_deviation));
  ASSERT_EQ(
    test_constants::kTestDuration.count() / test_constants::kMeasurePeriod.count(),
    data.sample_count);

  const bool stop_success = test_periodic_measurer_->Stop();
  ASSERT_TRUE(stop_success);
  ASSERT_FALSE(test_periodic_measurer_->IsStarted());

  int times_published = test_periodic_measurer_->GetNumPublished();
  ASSERT_EQ(
    test_constants::kTestDuration.count() / kDontPublishDuringTest.count(), times_published);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
