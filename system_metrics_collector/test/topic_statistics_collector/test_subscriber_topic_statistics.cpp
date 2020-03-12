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
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "../system_metrics_collector/test_constants.hpp"
#include "system_metrics_collector/collector.hpp"
#include "system_metrics_collector/constants.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.cpp"

using lifecycle_msgs::msg::State;

namespace
{
constexpr const int64_t kConstantTimestamp = 1000000;
constexpr const char kPublishPeriodParam[] = "publish_period";
constexpr const std::chrono::milliseconds kTestDuration{250};
constexpr const char kTestNodeName[] = "test_periodic_node";
constexpr const char kTestMetricName[] = "test_metric_name";
constexpr const char kTestMetricUnit[] = "test_unit";
constexpr const char kTestTopic[] = "/test_topic";
}  // namespace

class TestSubscriberTopicStatistics
  : public topic_statistics_collector::SubscriberTopicStatistics<
    sensor_msgs::msg::Imu>
{
public:
  TestSubscriberTopicStatistics(const std::string & name, const rclcpp::NodeOptions & options)
  : SubscriberTopicStatistics<sensor_msgs::msg::Imu>{name, options, kTestTopic} {}

  ~TestSubscriberTopicStatistics() = default;

  std::string GetMetricName() const override
  {
    return kTestMetricName;
  }

  const std::string & GetMetricUnit() const override
  {
    static const std::string unit_name{kTestMetricUnit};
    return unit_name;
  }

  /**
   * Return true if the lifecycle publisher is activated, false if null or not activated.
   *
   * @return
   */
  bool IsPublisherActivated() const
  {
    return publisher_ != nullptr && publisher_->is_activated();
  }

  int GetNumPublished() const
  {
    return times_published_;
  }

private:
  /**
   * Test publish for the fixture.
   */
  void PublishStatisticMessage() override
  {
    ++times_published_;
  }

  std::atomic<int> times_published_{0};
};

/**
 * Node to publish messages on the test topic to trigger subscriber callback.
*/
class ImuMessagePublisher : public rclcpp::Node
{
public:
  ImuMessagePublisher()
  : Node("imu_publisher"), publisher_(nullptr)
  {
    auto qos_options = rclcpp::QoS(rclcpp::KeepAll());
    auto publisher_options = rclcpp::PublisherOptions();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      kTestTopic,
      qos_options,
      publisher_options);
  }

  ~ImuMessagePublisher() = default;

  void publish(std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {
    publisher_->publish(*msg);
  }

private:
  typename rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

/**
 * Test fixture to test a SubscriberTopicStatistics node
 */
class SubscriberTopicStatisticsTestFixture : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(kPublishPeriodParam, kDontPublishDuringTest.count());

    test_periodic_publisher_ = std::make_shared<TestSubscriberTopicStatistics>(
      kTestNodeName, options);

    const moving_average_statistics::StatisticData data =
      test_periodic_publisher_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_periodic_publisher_->shutdown();
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_periodic_publisher_->get_current_state().id());
    EXPECT_FALSE(test_periodic_publisher_->IsStarted());
    EXPECT_FALSE(test_periodic_publisher_->IsPublisherActivated());

    test_periodic_publisher_.reset();
    rclcpp::shutdown();
  }

protected:
  // this test is not designed to have the statistics published and reset at any point of the test,
  // so this is defining the publish period to be something amply larger than the test duration
  // itself
  static constexpr std::chrono::milliseconds kDontPublishDuringTest = 2 * kTestDuration;
  std::shared_ptr<TestSubscriberTopicStatistics> test_periodic_publisher_;
};

/**
 * Fixture to bringup and teardown rclcpp
 */
class RclcppFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

std::shared_ptr<sensor_msgs::msg::Imu> GetImuMessageWithHeader(const int64_t timestamp)
{
  auto message = sensor_msgs::msg::Imu{};
  message.header = std_msgs::msg::Header{};
  message.header.stamp = rclcpp::Time{timestamp};
  return std::make_shared<sensor_msgs::msg::Imu>(message);
}

constexpr std::chrono::milliseconds SubscriberTopicStatisticsTestFixture::kDontPublishDuringTest;

TEST_F(SubscriberTopicStatisticsTestFixture, Sanity) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());

  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  test_periodic_publisher_->activate();
  ASSERT_TRUE(test_periodic_publisher_->IsStarted());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());


  ASSERT_EQ(
    "name=test_periodic_node, publishing_topic=/system_metrics,"
    " publish_period=500ms, started=true,"
    " avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_periodic_publisher_->GetStatusString());
}

TEST_F(SubscriberTopicStatisticsTestFixture, TestStartAndStop) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());


  test_periodic_publisher_->activate();
  ASSERT_TRUE(test_periodic_publisher_->IsStarted());
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());

  ImuMessagePublisher publisher;
  publisher.publish(GetImuMessageWithHeader(kConstantTimestamp));

  moving_average_statistics::StatisticData data =
    test_periodic_publisher_->GetStatisticsResults();
  ASSERT_EQ(1, data.sample_count);
  ASSERT_EQ(
    test_constants::kTestDuration.count() / test_constants::kMeasurePeriod.count(),
    data.max);
  ASSERT_FALSE(std::isnan(data.standard_deviation));
  ASSERT_EQ(
    test_constants::kTestDuration.count() / test_constants::kMeasurePeriod.count(),
    data.sample_count);

  test_periodic_publisher_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  int times_published = test_periodic_publisher_->GetNumPublished();
  ASSERT_EQ(
    test_constants::kTestDuration.count() / kDontPublishDuringTest.count(), times_published);
}

TEST_F(SubscriberTopicStatisticsTestFixture, TestLifecycleManually) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // configure the node
  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // activate the node
  test_periodic_publisher_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsStarted());
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());

  // deactivate the node
  test_periodic_publisher_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // shutdown happens in teardown
}

TEST_F(SubscriberTopicStatisticsTestFixture, TestLifecycleManually_reactivate) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // configure the node
  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // activate the node
  test_periodic_publisher_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsStarted());
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());

  // deactivate the node
  test_periodic_publisher_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsStarted());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // reactivate the node
  test_periodic_publisher_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsStarted());

  // shutdown happens in teardown
}

TEST_F(RclcppFixture, TestConstructorPublishPeriodValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kCollectPeriodParam,
    test_constants::kMeasurePeriod.count());
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    std::chrono::milliseconds{-1}.count());

  ASSERT_THROW(
    TestSubscriberTopicStatistics("throw", options),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(RclcppFixture, TestConstructorNodeNameValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kCollectPeriodParam,
    test_constants::kMeasurePeriod.count());
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    test_constants::kPublishPeriod.count());

  ASSERT_THROW(
    TestSubscriberTopicStatistics("", options),
    std::invalid_argument);
}
