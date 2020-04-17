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

#ifndef SYSTEM_METRICS_COLLECTOR__TEST_FUNCTIONS_HPP_
#define SYSTEM_METRICS_COLLECTOR__TEST_FUNCTIONS_HPP_

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "libstatistics_collector/moving_average_statistics/moving_average.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_memory_measurement_node.hpp"
#include "system_metrics_collector/utilities.hpp"


namespace test_functions
{
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;

using ExpectedStatistics =
  std::unordered_map<decltype(StatisticDataPoint::data_type), decltype(StatisticDataPoint::data)>;

/**
 * Convert input StatisticData to a map of StatisticDataPoint::data_type ->
 * StatisticDataPoint::data.
 *
 * @param src input data to convert
 * @return output ExpectedStatistics to use for testing.
 */
ExpectedStatistics StatisticDataToExpectedStatistics(
  const libstatistics_collector::moving_average_statistics::StatisticData & src)
{
  ExpectedStatistics expected{};
  expected[StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE] = src.average;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM] = src.min;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM] = src.max;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_STDDEV] = src.standard_deviation;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT] = src.sample_count;
  return expected;
}

/**
 * Check statistic equality. Fails for any unknown statistic type
 *
 * @param expected_stats expected data
 * @param actual data from a received ROS2 message
 */
void ExpectedStatisticEquals(
  const ExpectedStatistics & expected_stats,
  const statistics_msgs::msg::MetricsMessage & actual)
{
  for (const auto & stats_point : actual.statistics) {
    const auto type = stats_point.data_type;
    switch (type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
        EXPECT_DOUBLE_EQ(expected_stats.at(type), stats_point.data) << "unexpected sample count";
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
        EXPECT_DOUBLE_EQ(expected_stats.at(type), stats_point.data) << "unexpected average";
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
        EXPECT_DOUBLE_EQ(
          expected_stats.at(type), stats_point.data) << "unexpected min";
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
        EXPECT_DOUBLE_EQ(
          expected_stats.at(type), stats_point.data) << "unexpected max";
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
        EXPECT_DOUBLE_EQ(expected_stats.at(type), stats_point.data) << "unexpected stddev";
        break;
      default:
        FAIL() << "received unknown statistics type: " << std::dec <<
          static_cast<unsigned int>(type);
    }
  }
}

/**
 * Provide an interface to wait for a promise to be satisfied via its future.
 */
class PromiseSetter
{
public:
  /**
   * Reassign the promise member and return it's future. Acquires a mutex in order
   * to mutate member variables.
   *
   * @return the promise member's future, called upon PeriodicMeasurement
   */
  std::shared_future<bool> GetFuture()
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    use_future_ = true;
    promise_ = std::promise<bool>();
    return promise_.get_future();
  }

protected:
  /**
   * Set the promise to true, which signals the corresponding future. Acquires a mutex and sets
   * the promise to true iff GetFuture was invoked before this.
   */
  void SetPromise()
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    if (use_future_) {
      // only set if GetFuture was called
      promise_.set_value(true);
      use_future_ = false;          // the promise needs to be reassigned to set again
    }
  }

private:
  mutable std::mutex mutex_;
  std::promise<bool> promise_;
  bool use_future_{false};
};

/**
 * Node which listens for published MetricsMessages. This uses the PromiseSetter API
 * in order to signal, via a future, that rclcpp should stop spinning upon
 * message handling.
 */
class MetricsMessageSubscriber : public rclcpp::Node, public PromiseSetter
{
public:
  /**
   * Constucts a MetricsMessageSubscriber
   * @param name the node name
   * @param name the topic name
   */
  MetricsMessageSubscriber(const std::string & name, const std::string & topic_name)
  : rclcpp::Node(name)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {
        this->MetricsMessageCallback(*msg);
      };
    subscription_ = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(
      topic_name,
      0 /*history_depth*/,
      callback);
  }

  /**
   * Acquires a mutex in order to get the last message received member.
   * @return the last message received
   */
  MetricsMessage GetLastReceivedMessage() const
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    return last_received_message_;
  }

  /**
   * Return the number of messages received by this subscriber
   * @return the number of messages received by the subscriber callback
   */
  int GetNumberOfMessagesReceived() const
  {
    return num_messages_received_;
  }

private:
  /**
   * Subscriber callback. Aquires a mutex to set the last message received and
   * sets the promise to true
   * @param msg
   */
  void MetricsMessageCallback(const MetricsMessage & msg)
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    ++num_messages_received_;
    last_received_message_ = msg;
    PromiseSetter::SetPromise();
  }

  MetricsMessage last_received_message_;
  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
  mutable std::mutex mutex_;
  std::atomic<int> num_messages_received_{0};
};

}  // namespace test_functions

#endif  // SYSTEM_METRICS_COLLECTOR__TEST_FUNCTIONS_HPP_
