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
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "libstatistics_collector/moving_average_statistics/moving_average.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "system_metrics_collector/proc_cpu_data.hpp"
#include "system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"
#include "test_functions.hpp"

#include "rclcpp/rclcpp.hpp"


namespace
{
using libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
using libstatistics_collector::moving_average_statistics::StatisticData;
using lifecycle_msgs::msg::State;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using system_metrics_collector::ProcessStatCpuLine;
using test_constants::kProcSamples;

constexpr const char kTestCpuNodeName[] = "test_measure_linux_cpu";
constexpr const char kTestMetricName[] = "system_cpu_percent_used";
}  // namespace

/**
 * Test class used to fake out linux CPU measurements
 */
class TestLinuxCpuMeasurementNode : public system_metrics_collector::LinuxCpuMeasurementNode,
  public test_functions::PromiseSetter
{
public:
  TestLinuxCpuMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxCpuMeasurementNode{name, options} {}

  ~TestLinuxCpuMeasurementNode() override = default;

  // make this private method public for unit testing purposes
  double PeriodicMeasurement() override
  {
    // first measurement does not add to the sample count
    // cpu data requires two samples to compute a single measurement
    if (++count_ > 1) {
      count_ = 0;
      SetPromise();
    }
    return LinuxCpuMeasurementNode::PeriodicMeasurement();
  }

private:
  system_metrics_collector::ProcCpuData MakeSingleMeasurement() override
  {
    EXPECT_GT(kProcSamples.size(), measurement_index_);
    return ProcessStatCpuLine(kProcSamples[measurement_index_++]);
  }

  int measurement_index_{0};
  int count_{0};
};

class LinuxCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      test_constants::kMeasureCpuPeriod.count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      test_constants::kPublishCpuPeriod.count());

    test_measure_linux_cpu_ = std::make_shared<TestLinuxCpuMeasurementNode>(
      kTestCpuNodeName, options);

    ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

    const StatisticData data = test_measure_linux_cpu_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_cpu_->shutdown();
    EXPECT_EQ(State::PRIMARY_STATE_FINALIZED, test_measure_linux_cpu_->get_current_state().id());
    EXPECT_FALSE(test_measure_linux_cpu_->IsStarted());

    test_measure_linux_cpu_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxCpuMeasurementNode> test_measure_linux_cpu_;
};

TEST_F(LinuxCpuMeasurementTestFixture, TestManualMeasurement)
{
  // first measurement caches
  double cpu_active_percentage = test_measure_linux_cpu_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_measure_linux_cpu_->PeriodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::kCpuActiveProcSample_0_1, cpu_active_percentage);
}

/**
 * Test the lifecycle and check that measurements can be made.
 */
TEST_F(LinuxCpuMeasurementTestFixture, TestPeriodicMeasurement)
{
  ASSERT_NE(test_measure_linux_cpu_, nullptr);
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_cpu_->get_current_state().id());

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_cpu_->get_node_base_interface());

  // configure the node manually (lifecycle transition)
  test_measure_linux_cpu_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_cpu_->IsStarted());

  //
  // spin the node while activated and use the node's future to halt after the first measurement
  //
  ex.spin_until_future_complete(
    test_measure_linux_cpu_->GetFuture(), test_constants::kSpinTimeout);

  // expect that a single measurement will be made
  // sample 0 and 1
  auto data = test_measure_linux_cpu_->GetStatisticsResults();
  auto expected_cpu_active = ComputeCpuActivePercentage(
    ProcessStatCpuLine(kProcSamples[0]),
    ProcessStatCpuLine(kProcSamples[1]));

  EXPECT_EQ(1, data.sample_count);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0.0, data.standard_deviation);  // only one sample so 0

  //
  // spin the node with it deactivated
  //
  test_measure_linux_cpu_->deactivate();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

  // use the dummy future as the test_measure_linux_cpu_ promise won't be set
  ex.spin_until_future_complete(dummy_future, test_constants::kSpinTimeout);
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  data = test_measure_linux_cpu_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  // reactivate the node
  test_measure_linux_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_cpu_->get_current_state().id());

  //
  // spin the reactivated node and use the node's future to halt after the first measurement
  //
  ASSERT_TRUE(test_measure_linux_cpu_->IsStarted());
  ex.spin_until_future_complete(
    test_measure_linux_cpu_->GetFuture(), test_constants::kSpinTimeout);

  data = test_measure_linux_cpu_->GetStatisticsResults();

  expected_cpu_active = ComputeCpuActivePercentage(
    ProcessStatCpuLine(kProcSamples[2]),
    ProcessStatCpuLine(kProcSamples[3]));

  EXPECT_EQ(1, data.sample_count);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0.0, data.standard_deviation);  // only one sample so 0
}

/**
 * Test receipt of a single published message. Expectation is 3 periodic measurements
 * will be made and one message sent.
 */
TEST_F(LinuxCpuMeasurementTestFixture, TestPublishMessage)
{
  ASSERT_NE(test_measure_linux_cpu_, nullptr);
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_cpu_->get_current_state().id());

  const auto test_receive_measurements = std::make_shared<test_functions::MetricsMessageSubscriber>(
    "test_receive_measurements",
    system_metrics_collector::collector_node_constants::kStatisticsTopicName);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_cpu_->get_node_base_interface());
  ex.add_node(test_receive_measurements);

  // configure the node manually (lifecycle transition)
  test_measure_linux_cpu_->configure();
  ASSERT_EQ(State::PRIMARY_STATE_INACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_cpu_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_cpu_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_cpu_->IsStarted());

  // spin the node while activated and use the node's future to halt
  // after the first published message
  ex.spin_until_future_complete(
    test_receive_measurements->GetFuture(), test_constants::kPublishTestTimeout);
  EXPECT_EQ(test_receive_measurements->GetNumberOfMessagesReceived(), 1);

  // generate expected data: expectation is that 6 samples are taken within the publish
  // time frame
  MovingAverageStatistics expected_moving_average;
  for (int i = 0; i < kProcSamples.size() - 1; i++) {
    const auto d = ComputeCpuActivePercentage(
      ProcessStatCpuLine(kProcSamples[i]),
      ProcessStatCpuLine(kProcSamples[i + 1]));
    expected_moving_average.AddMeasurement(d);
  }

  const auto expected_stats = test_functions::StatisticDataToExpectedStatistics(
    expected_moving_average.GetStatistics());

  // check expected received message
  const auto received_message = test_receive_measurements->GetLastReceivedMessage();

  EXPECT_EQ(std::string(kTestCpuNodeName), received_message.measurement_source_name);
  EXPECT_EQ(std::string(kTestMetricName), received_message.metrics_source);
  EXPECT_EQ(
    std::string(system_metrics_collector::collector_node_constants::kPercentUnitName),
    received_message.unit);

  test_functions::ExpectedStatisticEquals(expected_stats, received_message);
}
