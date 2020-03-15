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
#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "../system_metrics_collector/test_constants.hpp"
#include "system_metrics_collector/collector.hpp"
#include "system_metrics_collector/constants.hpp"
#include "topic_statistics_collector/constants.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.cpp"
#include "topic_statistics_collector/topic_statistics_collector.hpp"

using lifecycle_msgs::msg::State;

namespace
{
constexpr const int64_t kAnyTimestamp = 1000000;
constexpr const char kPublishPeriodParam[] = "publish_period";
constexpr const std::chrono::milliseconds kTestDuration{250};
constexpr const char kTestNodeName[] = "test_periodic_node";
constexpr const char kCollectStatsTopicName[] = "collect_topic_name";
constexpr const char kTestTopicName[] = "/test_topic";
constexpr const uint64_t kTimesCallbackCalled = 10;
}  // namespace

class TestSubscriberTopicStatisticsNode
  : public topic_statistics_collector::SubscriberTopicStatisticsNode<
    sensor_msgs::msg::Imu>
{
public:
  TestSubscriberTopicStatisticsNode(const std::string & name, const rclcpp::NodeOptions & options)
  : SubscriberTopicStatisticsNode<sensor_msgs::msg::Imu>{name, options} {}

  ~TestSubscriberTopicStatisticsNode() = default;

  /**
   * Return true if the lifecycle publisher is activated, false if null or not activated.
   *
   * @return true if the publisher is active; false otherwise
   */
  bool IsPublisherActivated() const
  {
    return publisher_ != nullptr && publisher_->is_activated();
  }

  /**
   * Return number of times MetricsMessages are published.
   *
   * @return count of messages published
   */
  int GetNumPublished() const
  {
    return times_published_;
  }

  /**
   * Return the container holding topic statistics collector.
   *
   * @return data streucture containing the collectors
   */
  std::vector<std::shared_ptr<topic_statistics_collector::TopicStatisticsCollector<
      sensor_msgs::msg::Imu>>> GetCollectors() const
  {
    return statistics_collectors_;
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
 * Node to publish messages on the test topic to trigger subscriber callbacks.
*/
class ImuMessagePublisher : public rclcpp::Node
{
public:
  ImuMessagePublisher()
  : Node("imu_publisher"), publisher_(nullptr)
  {
    auto qos_options = rclcpp::QoS(rclcpp::KeepAll());
    auto publisher_options = rclcpp::PublisherOptions();

    publisher_ = create_publisher<sensor_msgs::msg::Imu>(kTestTopicName, 10);
  }

  ~ImuMessagePublisher() = default;

  /**
   * Publish a  single message.
   */
  void publish(std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {
    publisher_->publish(*msg);
  }

private:
  typename rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

/**
 * Test fixture to test a SubscriberTopicStatisticsNode node
 */
class SubscriberTopicStatisticsNodeTestFixture : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(kPublishPeriodParam, kDontPublishDuringTest.count());
    options.append_parameter_override(kCollectStatsTopicName, kTestTopicName);

    test_periodic_publisher_ = std::make_shared<TestSubscriberTopicStatisticsNode>(
      kTestNodeName, options);

    const auto collectors = test_periodic_publisher_->GetCollectors();
    ASSERT_FALSE(collectors.empty());
    for (const auto & collector : collectors) {
      auto data = collector->GetStatisticsResults();
      ASSERT_TRUE(std::isnan(data.average));
      ASSERT_TRUE(std::isnan(data.min));
      ASSERT_TRUE(std::isnan(data.max));
      ASSERT_TRUE(std::isnan(data.standard_deviation));
      ASSERT_EQ(0, data.sample_count);
    }
  }

  void TearDown() override
  {
    test_periodic_publisher_->shutdown();
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_periodic_publisher_->get_current_state().id());
    EXPECT_FALSE(test_periodic_publisher_->IsPublisherActivated());

    test_periodic_publisher_.reset();
    rclcpp::shutdown();
  }

protected:
  // this test is not designed to have the statistics published and reset at any point of the test,
  // so this is defining the publish period to be something amply larger than the test duration
  // itself
  static constexpr std::chrono::milliseconds kDontPublishDuringTest = 2 * kTestDuration;
  std::shared_ptr<TestSubscriberTopicStatisticsNode> test_periodic_publisher_;
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

/**
 * Generate an IMU message with a valid header value.
  *
  * @return shared_ptr to an IMU message
 */
std::shared_ptr<sensor_msgs::msg::Imu> GetImuMessageWithHeader()
{
  auto message = sensor_msgs::msg::Imu{};
  message.header = std_msgs::msg::Header{};
  message.header.stamp = rclcpp::Time{kAnyTimestamp};
  return std::make_shared<sensor_msgs::msg::Imu>(message);
}

constexpr std::chrono::milliseconds
SubscriberTopicStatisticsNodeTestFixture::kDontPublishDuringTest;

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestStart) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_FALSE(collector->IsStarted());
  }

  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  test_periodic_publisher_->activate();
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_TRUE(collector->IsStarted());
  }
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestSubscriptionCallbackAndPublish) {
  test_periodic_publisher_->configure();
  test_periodic_publisher_->activate();

  ImuMessagePublisher publisher;
  const auto msg = GetImuMessageWithHeader();
  for (int i = 0; i < kTimesCallbackCalled; ++i) {
    publisher.publish(msg);
  }

  moving_average_statistics::StatisticData data;
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    data = collector->GetStatisticsResults();
    ASSERT_EQ(kTimesCallbackCalled, data.sample_count);
    ASSERT_FALSE(std::isnan(data.average));
    ASSERT_FALSE(std::isnan(data.min));
    ASSERT_FALSE(std::isnan(data.max));
    ASSERT_FALSE(std::isnan(data.standard_deviation));
  }

  int times_published = test_periodic_publisher_->GetNumPublished();
  ASSERT_EQ(
    test_constants::kTestDuration.count() / kDontPublishDuringTest.count(), times_published);
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestStop) {
  test_periodic_publisher_->configure();
  test_periodic_publisher_->activate();

  test_periodic_publisher_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_FALSE(collector->IsStarted());
  }
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestLifecycleManually_reactivate) {
  ASSERT_NE(test_periodic_publisher_, nullptr);
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_FALSE(collector->IsStarted());
  }
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_periodic_publisher_->get_current_state().id());
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // configure the node
  test_periodic_publisher_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_FALSE(collector->IsStarted());
  }
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // activate the node
  test_periodic_publisher_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_TRUE(collector->IsStarted());
  }
  ASSERT_TRUE(test_periodic_publisher_->IsPublisherActivated());

  // deactivate the node
  test_periodic_publisher_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_periodic_publisher_->get_current_state().id());
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_FALSE(collector->IsStarted());
  }
  ASSERT_FALSE(test_periodic_publisher_->IsPublisherActivated());

  // reactivate the node
  test_periodic_publisher_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_periodic_publisher_->get_current_state().id());
  for (const auto & collector : test_periodic_publisher_->GetCollectors()) {
    ASSERT_TRUE(collector->IsStarted());
  }
}

TEST_F(RclcppFixture, TestConstructorNodeNameValidation) {
  rclcpp::NodeOptions options;

  ASSERT_THROW(
    TestSubscriberTopicStatisticsNode("", options),
    std::invalid_argument);
}

TEST_F(RclcppFixture, TestConstructorPublishPeriodValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    std::chrono::milliseconds{-1}.count());

  ASSERT_THROW(
    TestSubscriberTopicStatisticsNode("throw", options),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(RclcppFixture, TestConstructorTopicNameValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    topic_statistics_collector::topic_statistics_constants::kCollectStatsTopicName,
    std::string());

  ASSERT_THROW(
    TestSubscriberTopicStatisticsNode("throw", options),
    std::invalid_argument);
}
