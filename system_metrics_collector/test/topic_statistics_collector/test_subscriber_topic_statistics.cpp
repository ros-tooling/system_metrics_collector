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

#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/topic_statistics_collector.hpp"
#include "system_metrics_collector/constants.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.hpp"
#include "../system_metrics_collector/test_functions.hpp"

using lifecycle_msgs::msg::State;
using libstatistics_collector::moving_average_statistics::StatisticData;
namespace constants =
  libstatistics_collector::topic_statistics_collector::topic_statistics_constants;

namespace
{
constexpr const int64_t kAnyTimestamp = 1000000;
constexpr const std::chrono::milliseconds kTestDuration{250};
constexpr const char kStatsCollectorNodeName[] = "topic_stats_node";
constexpr const char kTestTopicName[] = "/test_topic";
constexpr const uint64_t kTimesCallbackCalled{10u};
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
   * Check if individual topic statistics collectors are started.
   *
   * @return true if all the collectoers are started, false otherwise
   */
  bool AreCollectorsStarted() const
  {
    for (const auto & collector : statistics_collectors_) {
      if (!collector->IsStarted()) {
        return false;
      }
    }
    return true;
  }

  /**
   * Return number of collectors owned by this node.
   *
   * @return number of collectors now_nanoseconds*/
  size_t GetCollectorCount() const
  {
    return statistics_collectors_.size();
  }

  /**
   * Return the statistic data collected by all individual collectors in this node.
   *
   * @return statistic data collected by all collectors
   */
  std::vector<StatisticData> GetCollectorData() const
  {
    std::vector<StatisticData> data;
    for (const auto & collector : statistics_collectors_) {
      data.push_back(collector->GetStatisticsResults());
    }
    return data;
  }

  /**
   * Spin a MetricsMessageSubscriber node until the desired MetricsMessages are received.
   */
  void SpinUntilMessageReceived()
  {
    const auto receive_messages = std::make_shared<test_functions::MetricsMessageSubscriber>(
      "receive_messages");

    rclcpp::executors::SingleThreadedExecutor ex;
    ex.add_node(this->get_node_base_interface());
    ex.add_node(receive_messages);

    ex.spin_until_future_complete(
      receive_messages->GetFuture(), kTestDuration);
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
    publisher_ = create_publisher<sensor_msgs::msg::Imu>(kTestTopicName, 10);
  }

  ~ImuMessagePublisher() = default;

  /**
   * Publish a  single IMU data message.
   */
  void publish(std::shared_ptr<sensor_msgs::msg::Imu> msg)
  {
    publisher_->publish(*msg);
  }

  /**
   * Get the number of subscribers subscribed to the ImuPublisher topic.
   *
   * @return number of subscriptions
   */
  uint64_t getSubscriptionCount() const
  {
    return publisher_->get_subscription_count();
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
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      kVeryLongPublishPeriod.count());
    options.append_parameter_override(constants::kCollectStatsTopicNameParam, kTestTopicName);

    imu_publisher_ = std::make_shared<ImuMessagePublisher>();
    EXPECT_EQ(imu_publisher_->getSubscriptionCount(), 0);

    test_topic_stats_node_ = std::make_shared<TestSubscriberTopicStatisticsNode>(
      kStatsCollectorNodeName, options);

    EXPECT_GT(test_topic_stats_node_->GetCollectorCount(), 0);

    const auto all_collected_data = test_topic_stats_node_->GetCollectorData();
    for (const auto & data : all_collected_data) {
      EXPECT_TRUE(std::isnan(data.average));
      EXPECT_TRUE(std::isnan(data.min));
      EXPECT_TRUE(std::isnan(data.max));
      EXPECT_TRUE(std::isnan(data.standard_deviation));
      EXPECT_EQ(0, data.sample_count);
    }
  }

  void TearDown() override
  {
    test_topic_stats_node_->shutdown();
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_topic_stats_node_->get_current_state().id());
    EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

    test_topic_stats_node_.reset();
    imu_publisher_.reset();
    rclcpp::shutdown();
  }

protected:
  // this test is not designed to have the statistics published and reset at any point of the test,
  // so this is defining the publish period to be something amply larger than the test duration
  // itself
  static constexpr std::chrono::milliseconds kVeryLongPublishPeriod = 2 * kTestDuration;
  std::shared_ptr<TestSubscriberTopicStatisticsNode> test_topic_stats_node_;
  std::shared_ptr<ImuMessagePublisher> imu_publisher_;
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
SubscriberTopicStatisticsNodeTestFixture::kVeryLongPublishPeriod;

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestStartAndStop) {
  EXPECT_NE(test_topic_stats_node_, nullptr);
  EXPECT_FALSE(test_topic_stats_node_->AreCollectorsStarted());

  EXPECT_EQ(State::PRIMARY_STATE_UNCONFIGURED, test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

  test_topic_stats_node_->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

  test_topic_stats_node_->activate();
  EXPECT_TRUE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_TRUE(test_topic_stats_node_->IsPublisherActivated());

  test_topic_stats_node_->deactivate();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestSubscriptionCallback) {
  test_topic_stats_node_->configure();
  test_topic_stats_node_->activate();

  EXPECT_TRUE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_TRUE(test_topic_stats_node_->IsPublisherActivated());

  const auto msg = GetImuMessageWithHeader();
  for (int i = 0; i < kTimesCallbackCalled; ++i) {
    imu_publisher_->publish(msg);
  }

  test_topic_stats_node_->SpinUntilMessageReceived();

  const auto all_collected_data = test_topic_stats_node_->GetCollectorData();
  for (const auto & data : all_collected_data) {
    EXPECT_GT(data.sample_count, 0);
    EXPECT_FALSE(std::isnan(data.average));
    EXPECT_FALSE(std::isnan(data.min));
    EXPECT_FALSE(std::isnan(data.max));
    EXPECT_FALSE(std::isnan(data.standard_deviation));
  }

  int times_published = test_topic_stats_node_->GetNumPublished();
  EXPECT_EQ(
    kTestDuration.count() / kVeryLongPublishPeriod.count(), times_published);
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestLifecycleManually_reactivate) {
  EXPECT_NE(test_topic_stats_node_, nullptr);
  EXPECT_FALSE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

  // configure the node
  test_topic_stats_node_->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

  // activate the node
  test_topic_stats_node_->activate();
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_TRUE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_TRUE(test_topic_stats_node_->IsPublisherActivated());

  // deactivate the node
  test_topic_stats_node_->deactivate();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_FALSE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());

  // reactivate the node
  test_topic_stats_node_->activate();
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_topic_stats_node_->get_current_state().id());
  EXPECT_TRUE(test_topic_stats_node_->AreCollectorsStarted());
}

TEST_F(RclcppFixture, TestConstructorNodeNameValidation) {
  rclcpp::NodeOptions options;

  EXPECT_THROW(
    TestSubscriberTopicStatisticsNode("", options),
    std::invalid_argument);
}

TEST_F(RclcppFixture, TestConstructorPublishPeriodValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    std::chrono::milliseconds{-1}.count());

  EXPECT_THROW(
    TestSubscriberTopicStatisticsNode("throw", options),
    rclcpp::exceptions::InvalidParameterValueException);
}

TEST_F(RclcppFixture, TestConstructorTopicNameValidation) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(constants::kCollectStatsTopicNameParam, std::string());

  EXPECT_THROW(
    TestSubscriberTopicStatisticsNode("throw", options),
    std::invalid_argument);
}

TEST_F(RclcppFixture, TestMetricsMessagePublisher) {
  rclcpp::NodeOptions options;
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    std::chrono::milliseconds{kTestDuration}.count());
  options.append_parameter_override(constants::kCollectStatsTopicNameParam, kTestTopicName);

  auto test_node = std::make_shared<TestSubscriberTopicStatisticsNode>(
    "TestMetricsMessagePublisher",
    options);
  test_node->configure();
  test_node->activate();

  auto publisher_node = std::make_shared<ImuMessagePublisher>();
  const auto msg = GetImuMessageWithHeader();
  for (int i = 0; i < kTimesCallbackCalled; ++i) {
    publisher_node->publish(msg);
  }

  // After spinning, test that MetricsMessage is published and collected values are cleared.
  test_node->SpinUntilMessageReceived();

  EXPECT_EQ(test_node->GetNumPublished(), 1);

  const auto all_collected_data = test_node->GetCollectorData();
  for (const auto & data : all_collected_data) {
    EXPECT_EQ(data.sample_count, 0);
    EXPECT_TRUE(std::isnan(data.average));
    EXPECT_TRUE(std::isnan(data.min));
    EXPECT_TRUE(std::isnan(data.max));
    EXPECT_TRUE(std::isnan(data.standard_deviation));
  }
}
