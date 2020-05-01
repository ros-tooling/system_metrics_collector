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

#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "libstatistics_collector/topic_statistics_collector/topic_statistics_collector.hpp"
#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/msg/dummy_message.hpp"
#include "topic_statistics_collector/subscriber_topic_statistics.hpp"
#include "../system_metrics_collector/test_functions.hpp"

namespace
{
using lifecycle_msgs::msg::State;
using libstatistics_collector::moving_average_statistics::StatisticData;
namespace constants =
  libstatistics_collector::topic_statistics_collector::topic_statistics_constants;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using DummyMessage = system_metrics_collector::msg::DummyMessage;

constexpr const int64_t kAnyTimestamp = 1000000;
constexpr const std::chrono::seconds kTestDuration{10};
constexpr const std::chrono::milliseconds kDummyPublishPeriod{100};
constexpr const std::chrono::milliseconds kTopicPublishPeriod{1000};
constexpr const char kStatsCollectorNodeName[] = "topic_stats_node";
constexpr const char kTestTopicName[] = "/test_topic";
constexpr const uint64_t kTimesCallbackCalled{10u};
}  // namespace

class TestSubscriberTopicStatisticsNode
  : public topic_statistics_collector::SubscriberTopicStatisticsNode<
    DummyMessage>, public test_functions::PromiseSetter
{
public:
  TestSubscriberTopicStatisticsNode(const std::string & name, const rclcpp::NodeOptions & options)
  : SubscriberTopicStatisticsNode<DummyMessage>{name, options} {}

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
  std::vector<StatisticData> GetCurrentCollectorData() const
  {
    std::vector<StatisticData> data;
    for (const auto & collector : statistics_collectors_) {
      data.push_back(collector->GetStatisticsResults());
    }
    return data;
  }

  const std::vector<StatisticData> & GetPublishedCollectorData() const
  {
    return last_published_data_;
  }

private:
  /**
   * Test publish for the fixture.
   */
  void PublishStatisticMessage() override
  {
    last_published_data_ = this->GetCurrentCollectorData();
    ++times_published_;
    test_functions::PromiseSetter::SetPromise();
  }

  std::atomic<int> times_published_{0};
  std::vector<StatisticData> last_published_data_;
};

/**
 * Node to publish messages on the test topic to trigger subscriber callbacks.
*/
class DummyMessagePublisher : public rclcpp::Node
{
public:
  DummyMessagePublisher()
  : Node("dummy_publisher"), publisher_(nullptr)
  {
    publisher_ = create_publisher<DummyMessage>(kTestTopicName, 10);
    publish_timer_ = this->create_wall_timer(
      kDummyPublishPeriod, [this]() {
        this->PublishMessage();
      });
  }

  ~DummyMessagePublisher() = default;

  int GetNumberPublished() const
  {
    return number_published_;
  }

  /**
   * Publish a  single DummyMessage.
   */
  void PublishMessage()
  {
    ++number_published_;
    auto msg = DummyMessage{};
    msg.header.stamp = rclcpp::Time{kAnyTimestamp};
    publisher_->publish(msg);
  }

  /**
   * Get the number of subscribers subscribed to the DummyMessage publisher topic.
   *
   * @return number of subscriptions
   */
  uint64_t getSubscriptionCount() const
  {
    return publisher_->get_subscription_count();
  }

private:
  typename rclcpp::Publisher<DummyMessage>::SharedPtr publisher_;
  std::atomic<int> number_published_{0};
  rclcpp::TimerBase::SharedPtr publish_timer_;
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
    // set the topic stat publishing period
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      kTopicPublishPeriod.count());
    // set the topic to collect statistics (listen)
    std::vector<std::string> test_topicname = {kTestTopicName};
    options.append_parameter_override(constants::kCollectStatsTopicNameParam, test_topicname);

    dummy_publisher_ = std::make_shared<DummyMessagePublisher>();
    EXPECT_EQ(dummy_publisher_->getSubscriptionCount(), 0);

    test_topic_stats_node_ = std::make_shared<TestSubscriberTopicStatisticsNode>(
      kStatsCollectorNodeName, options);

    EXPECT_GT(test_topic_stats_node_->GetCollectorCount(), 0);

    const auto all_collected_data = test_topic_stats_node_->GetCurrentCollectorData();
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
    if (test_topic_stats_node_->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
      test_topic_stats_node_->shutdown();
      EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_topic_stats_node_->get_current_state().id());
      EXPECT_FALSE(test_topic_stats_node_->IsPublisherActivated());
    }

    test_topic_stats_node_.reset();
    dummy_publisher_.reset();
    rclcpp::shutdown();
  }

protected:
  // this test is not designed to have the statistics published and reset at any point of the test,
  // so this is defining the publish period to be something amply larger than the test duration
  // itself
  static constexpr std::chrono::milliseconds kVeryLongPublishPeriod{500};
  std::shared_ptr<TestSubscriberTopicStatisticsNode> test_topic_stats_node_;
  std::shared_ptr<DummyMessagePublisher> dummy_publisher_;
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
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_topic_stats_node_->get_current_state().id());

  test_topic_stats_node_->activate();
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_topic_stats_node_->get_current_state().id());

  EXPECT_TRUE(test_topic_stats_node_->AreCollectorsStarted());
  EXPECT_TRUE(test_topic_stats_node_->IsPublisherActivated());

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_topic_stats_node_->get_node_base_interface());
  ex.add_node(dummy_publisher_);

  // future fires for a single publish, otherwise long timeout
  ex.spin_until_future_complete(test_topic_stats_node_->GetFuture(), kTestDuration);

  // check we have at least published once
  EXPECT_EQ(test_topic_stats_node_->GetNumPublished(), 1);
  // check that dummy data was actually published
  EXPECT_GT(dummy_publisher_->GetNumberPublished(), 0);

  const auto all_collected_data = test_topic_stats_node_->GetPublishedCollectorData();

  EXPECT_GT(all_collected_data.size(), 0);

  for (const auto & data : all_collected_data) {
    EXPECT_GT(data.sample_count, 1);
    EXPECT_FALSE(std::isnan(data.average));
    EXPECT_FALSE(std::isnan(data.min));
    EXPECT_FALSE(std::isnan(data.max));
    EXPECT_FALSE(std::isnan(data.standard_deviation));
  }
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
  options.append_parameter_override(
    constants::kCollectStatsTopicNameParam,
    std::vector<std::string>());

  EXPECT_THROW(
    TestSubscriberTopicStatisticsNode("throw", options),
    std::invalid_argument);
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestConstructorTopicNameVectorValues) {
  rclcpp::NodeOptions options;
  std::vector<std::string> test_topicname = {kTestTopicName};
  options.append_parameter_override(
    constants::kCollectStatsTopicNameParam,
    test_topicname);
  topic_statistics_collector::SubscriberTopicStatisticsNode<DummyMessage> topic_statistics(
    "TestConstructorTopicNameVectorValues",
    options);
  EXPECT_EQ(test_topicname, topic_statistics.GetCollectTopicName());
}

TEST_F(SubscriberTopicStatisticsNodeTestFixture, TestMetricsMessagePublisher) {
  rclcpp::NodeOptions options;
  // set the topic stat publishing period
  options.append_parameter_override(
    system_metrics_collector::collector_node_constants::kPublishPeriodParam,
    kTopicPublishPeriod.count());
  // set the topic to collect statistics (listen)
  std::vector<std::string> test_topicname = {kTestTopicName};
  options.append_parameter_override(constants::kCollectStatsTopicNameParam, test_topicname);

  const auto receive_messages = std::make_shared<test_functions::MetricsMessageSubscriber>(
    "TestMetricsMessagePublisher_listener",
    system_metrics_collector::collector_node_constants::kStatisticsTopicName);

  auto test_node =
    std::make_shared<topic_statistics_collector::SubscriberTopicStatisticsNode<DummyMessage>>(
    "TestMetricsMessagePublisher",
    options);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_node->get_node_base_interface());
  ex.add_node(dummy_publisher_);
  ex.add_node(receive_messages);

  test_node->configure();
  EXPECT_EQ(State::PRIMARY_STATE_INACTIVE, test_node->get_current_state().id());

  test_node->activate();
  EXPECT_EQ(State::PRIMARY_STATE_ACTIVE, test_node->get_current_state().id());

  // wait until a metrics message was received
  ex.spin_until_future_complete(
    receive_messages->GetFuture(),
    kTestDuration);

  // check that dummy data was actually published
  EXPECT_GT(dummy_publisher_->GetNumberPublished(), 0);
  // check that we actually received a message
  EXPECT_EQ(receive_messages->GetNumberOfMessagesReceived(), 1);

  // check the received message
  const auto received_message = receive_messages->GetLastReceivedMessage();
  for (const auto & stats_point : received_message.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
        EXPECT_GT(dummy_publisher_->GetNumberPublished(), 1) << "unexpected sample count";
        break;
      default:
        // do nothing, we don't check statistics validity here but at a minimum
        // check that the sample count is at least non-zero
        break;
    }
  }
}
