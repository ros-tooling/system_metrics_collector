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

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "statistics_msgs/msg/metrics_message.hpp"
#include "statistics_msgs/msg/statistic_data_type.hpp"

#include "libstatistics_collector/moving_average_statistics/moving_average.hpp"

#include "system_metrics_collector/constants.hpp"
#include "system_metrics_collector/linux_process_cpu_measurement_node.hpp"
#include "system_metrics_collector/proc_cpu_data.hpp"
#include "system_metrics_collector/utilities.hpp"

#include "test_constants.hpp"
#include "test_functions.hpp"


namespace
{
using libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
using libstatistics_collector::moving_average_statistics::StatisticData;
using lifecycle_msgs::msg::State;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using test_constants::kProcPidSamples;

constexpr const char kTestProcessCpuNodeName[] = "test_measure_linux_process_cpu";
}  // namespace

class MockLinuxProcessCpuMeasurementNode : public system_metrics_collector::
  LinuxProcessCpuMeasurementNode, public test_functions::PromiseSetter
{
public:
  MockLinuxProcessCpuMeasurementNode(const std::string & name, const rclcpp::NodeOptions & options)
  : LinuxProcessCpuMeasurementNode{name, options} {}

  /**
   * Exposes the protected member function for testing purposes.
   * See description for LinuxProcessCpuMeasurementNode::PeriodicMeasurement().
   *
   * @return percentage of CPU this process used
   */
  double PeriodicMeasurement() override
  {
    // first measurement does not add to the sample count
    // proc cpu data requires two samples to compute a single measurement
    if (++count_ > 1) {
      count_ = 0;
      PromiseSetter::SetPromise();
    }
    return LinuxProcessCpuMeasurementNode::PeriodicMeasurement();
  }

  /**
   * Exposes the protected member function for testing purposes.
   * See description for LinuxProcessCpuMeasurementNode::GetMetricName().
   *
   * @return a string of the name for this measured metric
   */
  std::string GetMetricName() const override
  {
    return LinuxProcessCpuMeasurementNode::GetMetricName();
  }

private:
  system_metrics_collector::ProcPidCpuData MakeSingleMeasurement() override
  {
    EXPECT_GT(kProcPidSamples.size(), measurement_index_);
    return kProcPidSamples[measurement_index_++];
  }

  int measurement_index_{0};
  int count_{0};
};

class LinuxProcessCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    using namespace std::chrono_literals;

    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kCollectPeriodParam,
      test_constants::kMeasureCpuPeriod.count());
    options.append_parameter_override(
      system_metrics_collector::collector_node_constants::kPublishPeriodParam,
      test_constants::kPublishCpuPeriod.count());

    test_measure_linux_process_cpu_ = std::make_shared<MockLinuxProcessCpuMeasurementNode>(
      kTestProcessCpuNodeName, options);

    ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());

    const StatisticData data = test_measure_linux_process_cpu_->GetStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_process_cpu_->shutdown();
    EXPECT_FALSE(test_measure_linux_process_cpu_->IsStarted());
    EXPECT_EQ(
      State::PRIMARY_STATE_FINALIZED,
      test_measure_linux_process_cpu_->get_current_state().id());

    test_measure_linux_process_cpu_.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<MockLinuxProcessCpuMeasurementNode> test_measure_linux_process_cpu_;
};

TEST_F(LinuxProcessCpuMeasurementTestFixture, TestGetMetricName) {
  const int pid = system_metrics_collector::GetPid();
  ASSERT_EQ(
    std::to_string(
      pid) + "_cpu_percent_used", test_measure_linux_process_cpu_->GetMetricName());
}

TEST_F(LinuxProcessCpuMeasurementTestFixture, TestManualMeasurement)
{
  // first measurement caches
  auto cpu_active_percentage = test_measure_linux_process_cpu_->PeriodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_measure_linux_process_cpu_->PeriodicMeasurement();
  ASSERT_DOUBLE_EQ(test_constants::kCpuActiveProcPidSample_0_1, cpu_active_percentage);
}

/**
 * Test the lifecycle and check that measurements can be made.
 */
TEST_F(LinuxProcessCpuMeasurementTestFixture, TestPeriodicMeasurement)
{
  ASSERT_NE(test_measure_linux_process_cpu_, nullptr);
  ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_process_cpu_->get_current_state().id());

  std::promise<bool> empty_promise;
  std::shared_future<bool> dummy_future = empty_promise.get_future();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_process_cpu_->get_node_base_interface());

  // configure the node manually (lifecycle transition)
  test_measure_linux_process_cpu_->configure();
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE,
    test_measure_linux_process_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_process_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_process_cpu_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_process_cpu_->IsStarted());

  //
  // spin the node while activated and use the node's future to halt after the first measurement
  //
  ex.spin_until_future_complete(
    test_measure_linux_process_cpu_->GetFuture(), test_constants::kSpinTimeout);

  // expect that a single measurement will be made
  // sample 0 and 1
  auto data = test_measure_linux_process_cpu_->GetStatisticsResults();
  auto expected_cpu_active = system_metrics_collector::ComputePidCpuActivePercentage(
    kProcPidSamples[0],
    kProcPidSamples[1]);

  EXPECT_EQ(1, data.sample_count);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.average);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.min);
  EXPECT_DOUBLE_EQ(expected_cpu_active, data.max);
  EXPECT_DOUBLE_EQ(0.0, data.standard_deviation);  // only one sample so 0

  //
  // spin the node with it deactivated
  //
  test_measure_linux_process_cpu_->deactivate();
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE,
    test_measure_linux_process_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());

  // use the dummy future as the test_measure_linux_process_cpu_ promise won't be set
  ex.spin_until_future_complete(dummy_future, test_constants::kSpinTimeout);
  // expectation is:
  // upon calling stop, samples are cleared, so GetStatisticsResults() would be NaNs
  data = test_measure_linux_process_cpu_->GetStatisticsResults();
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
  EXPECT_EQ(0, data.sample_count);

  // reactivate the node
  test_measure_linux_process_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_process_cpu_->get_current_state().id());

  //
  // spin the reactivated node and use the node's future to halt after the first measurement
  //
  ASSERT_TRUE(test_measure_linux_process_cpu_->IsStarted());
  ex.spin_until_future_complete(
    test_measure_linux_process_cpu_->GetFuture(), test_constants::kSpinTimeout);

  data = test_measure_linux_process_cpu_->GetStatisticsResults();

  expected_cpu_active = system_metrics_collector::ComputePidCpuActivePercentage(
    kProcPidSamples[2],
    kProcPidSamples[3]);

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
TEST_F(LinuxProcessCpuMeasurementTestFixture, TestPublishMessage)
{
  ASSERT_NE(test_measure_linux_process_cpu_, nullptr);
  ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());
  ASSERT_EQ(
    State::PRIMARY_STATE_UNCONFIGURED,
    test_measure_linux_process_cpu_->get_current_state().id());

  const auto test_receive_measurements = std::make_shared<test_functions::MetricsMessageSubscriber>(
    "test_receive_measurements",
    system_metrics_collector::collector_node_constants::kStatisticsTopicName);

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(test_measure_linux_process_cpu_->get_node_base_interface());
  ex.add_node(test_receive_measurements);

  // configure the node manually (lifecycle transition)
  test_measure_linux_process_cpu_->configure();
  ASSERT_EQ(
    State::PRIMARY_STATE_INACTIVE,
    test_measure_linux_process_cpu_->get_current_state().id());
  ASSERT_FALSE(test_measure_linux_process_cpu_->IsStarted());

  // activate the node manually (lifecycle transition): this allows collection and data publication
  test_measure_linux_process_cpu_->activate();
  ASSERT_EQ(State::PRIMARY_STATE_ACTIVE, test_measure_linux_process_cpu_->get_current_state().id());
  ASSERT_TRUE(test_measure_linux_process_cpu_->IsStarted());

  //
  // spin the node while activated and use the node's future to halt
  // after the first published message
  //
  ex.spin_until_future_complete(
    test_receive_measurements->GetFuture(), test_constants::kPublishTestTimeout);
  EXPECT_EQ(test_receive_measurements->GetNumberOfMessagesReceived(), 1);

  // generate expected data: expectation is that 6 samples are taken within the publish
  // time frame
  MovingAverageStatistics expected_moving_average;
  for (int i = 0; i < kProcPidSamples.size() - 1; i++) {
    const auto d = system_metrics_collector::ComputePidCpuActivePercentage(
      kProcPidSamples[i],
      kProcPidSamples[i + 1]);
    expected_moving_average.AddMeasurement(d);
  }

  const auto expected_stats = test_functions::StatisticDataToExpectedStatistics(
    expected_moving_average.GetStatistics());

  // check expected received message
  const auto received_message = test_receive_measurements->GetLastReceivedMessage();

  EXPECT_EQ(std::string(kTestProcessCpuNodeName), received_message.measurement_source_name);
  EXPECT_EQ(
    std::string(system_metrics_collector::collector_node_constants::kPercentUnitName),
    received_message.unit);

  test_functions::ExpectedStatisticEquals(expected_stats, received_message);
}
