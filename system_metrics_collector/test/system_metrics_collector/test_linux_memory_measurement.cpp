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

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "metrics_statistics_msgs/msg/metrics_message.hpp"
#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

#include "moving_average_statistics/moving_average.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_memory_measurement_node.hpp"
#include "system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"


using lifecycle_msgs::msg::State;
using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;
using moving_average_statistics::StatisticData;
using system_metrics_collector::ProcessMemInfoLines;
using moving_average_statistics::MovingAverageStatistics;

using ExpectedStatistics =
  std::unordered_map<decltype(StatisticDataPoint::data_type), decltype(StatisticDataPoint::data)>;

namespace
{
constexpr const char kTestNodeName[] = "test_measure_linux_memory";
constexpr const char kTestMetricName[] = "system_memory_percent_used";
constexpr const std::chrono::seconds kPublishTestTimeout{2};
constexpr const std::chrono::milliseconds kPublishPeriod{150};

const std::vector<std::string> kSamples = {
  "MemTotal:       16304208 kB\n"
  "MemFree:          845168 kB\n"
  "MemAvailable:    4840176 kB\n",

  "MemTotal:       16302048 kB\n"
  "MemFree:         9104952 kB\n"
  "MemAvailable:     239124 kB\n",

  "MemTotal:       16304208 kB\n"
  "MemFree:          826912 kB\n"
  "MemAvailable:    4837388 kB\n",
};
}  // namespace

ExpectedStatistics StatisticDataToExpectedStatistics(const StatisticData & src)
{
  ExpectedStatistics expected{};
  expected[StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE] = src.average;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM] = src.min;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM] = src.max;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_STDDEV] = src.standard_deviation;
  expected[StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT] = src.sample_count;
  return expected;
}

class PromiseSetter
{
public:
  /**
   * Reassign the promise member and return it's future.
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
  void SetPromise()
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    if (use_future_) {
      // only set if GetFuture was called
      promise_.set_value(true);
      use_future_ = false;    // the promise needs to be reassigned to set again
    }
  }

private:
  mutable std::mutex mutex_;
  std::promise<bool> promise_;
  bool use_future_{false};
};
/**
 *
 */
class TestLinuxMemoryMeasurementNode : public system_metrics_collector::LinuxMemoryMeasurementNode,
  public PromiseSetter
{
public:
  TestLinuxMemoryMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxMemoryMeasurementNode{name, options} {}
  ~TestLinuxMemoryMeasurementNode() override = default;

  /**
   *
   * @param test_data
   */
  void SetTestVector(const std::vector<std::string> & test_data)
  {
    test_vector_ = test_data;
    index_ = 0;
  }

  /**
   * Override to avoid calling methods involved in file i/o.
   *
   *
   */
  double PeriodicMeasurement() override
  {
    const auto to_return = ProcessMemInfoLines(test_vector_[index_]);
    if (++index_ >= test_vector_.size()) {
      index_ = 0;
    }
    PromiseSetter::SetPromise();
    return to_return;
  }

private:
  std::vector<std::string> test_vector_{""};
  std::atomic<int> index_{0};
};

/**
 * Node which listens for published MetricsMessages.
 */
class TestReceiveMemoryMeasurementNode : public rclcpp::Node, public PromiseSetter
{
public:
  explicit TestReceiveMemoryMeasurementNode(const std::string & name)
  : rclcpp::Node(name)          //, received_(false)
  {
    auto callback = [this](MetricsMessage::UniquePtr msg) {this->MetricsMessageCallback(*msg);};
    subscription_ = create_subscription<MetricsMessage,
        std::function<void(MetricsMessage::UniquePtr)>>(
      system_metrics_collector::collector_node_constants::kStatisticsTopicName,
      0 /*history_depth*/, callback);
  }

  MetricsMessage GetLastReceivedMessage()
  {
    return last_received_message_;   // todo locks
  }

private:
  void MetricsMessageCallback(const MetricsMessage & msg)
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    last_received_message_ = msg;
    PromiseSetter::SetPromise();
  }

  MetricsMessage last_received_message_;
  rclcpp::Subscription<MetricsMessage>::SharedPtr subscription_;
  mutable std::mutex mutex_;
};

/**
 *
 */
class LinuxMemoryMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      test_constants::kMeasurePeriod.count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      kPublishPeriod.count());

    test_measure_linux_memory_ = std::make_shared<TestLinuxMemoryMeasurementNode>(
      kTestNodeName, options);

    ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

    const moving_average_statistics::StatisticData data =
      test_measure_linux_memory_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_memory_->shutdown();
    EXPECT_FALSE(test_measure_linux_memory_->IsStarted());
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_measure_linux_memory_->get_current_state().id());

    test_measure_linux_memory_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxMemoryMeasurementNode> test_measure_linux_memory_;
};

TEST(LinuxMemoryMeasurementTest, TestReadInvalidFile)
{
  const auto s = system_metrics_collector::ReadFileToString("this_will_fail.txt");
  ASSERT_EQ("", s);
}

TEST_F(LinuxMemoryMeasurementTestFixture, testManualMeasurement) {
  double mem_used_percentage = test_measure_linux_memory_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(mem_used_percentage));

  const auto tv_valid = std::vector<std::string>{test_constants::kFullSample};
  test_measure_linux_memory_->SetTestVector(tv_valid);
  mem_used_percentage = test_measure_linux_memory_->PeriodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, mem_used_percentage);
}

TEST_F(LinuxMemoryMeasurementTestFixture, TestPeriodicMeasurement)
{
  ASSERT_NE(test_measure_linux_memory_, nullptr);
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_memory_->get_current_state().id());

  // set with a single valid sample
  const auto tv_valid = std::vector<std::string>{test_constants::kFullSample};
  test_measure_linux_memory_->SetTestVector(tv_valid);

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_memory_->get_node_base_interface());

  // configure the node manually (lifecycle transition)
  test_measure_linux_memory_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_memory_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_memory_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_memory_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_memory_->IsStarted());

  //
  // spin the node while activated and use the node's future to halt after the first measurement
  //
  ex.spin_until_future_complete(
    test_measure_linux_memory_->GetFuture(), test_constants::kSpinTimeout);

  // expect that a single measurement will be made
  auto data = test_measure_linux_memory_->GetStatisticsResults();
  EXPECT_EQ(1, data.sample_count);

  //
  // spin the node with it deactivated
  //
  test_measure_linux_memory_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_memory_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

  // use the dummy future as the test_measure_linux_memory_ promise won't be set
  ex.spin_until_future_complete(dummy_future, test_constants::kSpinTimeout);
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  // no MetricsMessages are published
  data = test_measure_linux_memory_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  // reactivate the node
  test_measure_linux_memory_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_memory_->get_current_state().id());

  //
  // spin the reactivated node and use the node's future to halt after the first measurement
  //
  ASSERT_TRUE(test_measure_linux_memory_->IsStarted());
  ex.spin_until_future_complete(
    test_measure_linux_memory_->GetFuture(), test_constants::kSpinTimeout);

  data = test_measure_linux_memory_->GetStatisticsResults();

  EXPECT_EQ(1, data.sample_count);
  EXPECT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, data.average);
  EXPECT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, data.min);
  EXPECT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, data.max);
  EXPECT_DOUBLE_EQ(0.0, data.standard_deviation);  // only one sample so 0
}

TEST_F(LinuxMemoryMeasurementTestFixture, TestPublishMessage)
{
  ASSERT_NE(test_measure_linux_memory_, nullptr);
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_memory_->get_current_state().id());

  // set with a single valid sample
  const auto tv_valid = std::vector<std::string>{test_constants::kFullSample};
  test_measure_linux_memory_->SetTestVector(kSamples);

  auto test_receive_measurements = std::make_shared<TestReceiveMemoryMeasurementNode>(
    "test_receive_measurements");

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_memory_->get_node_base_interface());
  ex.add_node(test_receive_measurements);

  // configure the node manually (lifecycle transition)
  test_measure_linux_memory_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_memory_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_memory_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_memory_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_memory_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_memory_->IsStarted());

  //
  // spin the node while activated and use the node's future to halt
  // after the first published message
  //
  ex.spin_until_future_complete(
    test_receive_measurements->GetFuture(), kPublishTestTimeout);

  MovingAverageStatistics expected_moving_average;
  for (const std::string sample : kSamples) {
    const auto d = ProcessMemInfoLines(sample);
    expected_moving_average.AddMeasurement(d);
  }

  const auto expected_stats = StatisticDataToExpectedStatistics(
    expected_moving_average.GetStatistics());

  // check expected received message
  const auto received_message = test_receive_measurements->GetLastReceivedMessage();
  EXPECT_EQ(kTestNodeName, received_message.measurement_source_name);
  EXPECT_EQ(kTestMetricName, received_message.metrics_source);
  EXPECT_EQ(
    system_metrics_collector::collector_node_constants::kPercentUnitName,
    received_message.unit);

  for (const StatisticDataPoint & stats_point : received_message.statistics) {
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
        FAIL() << "received unknown statistics type: " << std::to_string(type);
    }
  }
}
