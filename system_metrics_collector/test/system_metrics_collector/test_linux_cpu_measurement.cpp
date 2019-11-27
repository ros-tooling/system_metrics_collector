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

#include "../../src/system_metrics_collector/linux_cpu_measurement_node.hpp"
#include "../../src/system_metrics_collector/proc_cpu_data.hpp"

namespace
{
constexpr const char proc_sample_1[] =
  "cpu  22451232 118653 7348045 934943300 5378119 0 419114 0 0 0\n";
constexpr const char proc_sample_2[] =
  "cpu  22451360 118653 7348080 934949227 5378120 0 419117 0 0 0\n";
constexpr const std::chrono::milliseconds TEST_PERIOD =
  std::chrono::milliseconds(50);
constexpr const double CPU_ACTIVE_PERCENTAGE = 2.8002699055330633;
}  // namespace

class TestLinuxCpuMeasurementNode : public system_metrics_collector::LinuxCpuMeasurementNode
{
public:
  TestLinuxCpuMeasurementNode(
    const std::string & name,
    const std::chrono::milliseconds measurement_period,
    const std::string & publishing_topic)
  : LinuxCpuMeasurementNode(name, measurement_period, publishing_topic,
      PeriodicMeasurementNode::DEFAULT_PUBLISH_WINDOW)
  {}
  virtual ~TestLinuxCpuMeasurementNode() = default;

  double periodicMeasurement() override
  {
    LinuxCpuMeasurementNode::periodicMeasurement();
  }

private:
  system_metrics_collector::ProcCpuData makeSingleMeasurement() override
  {
    first = !first;

    if (first) {
      return system_metrics_collector::processLine(proc_sample_1);
    } else {
      return system_metrics_collector::processLine(proc_sample_2);
    }
  }

  bool first{false};
};

class LinuxCpuMeasurementTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    test_measure_linux_cpu = std::make_shared<TestLinuxCpuMeasurementNode>("test_periodic_node",
        TEST_PERIOD, "test_topic");

    ASSERT_FALSE(test_measure_linux_cpu->isStarted());

    const moving_average_statistics::StatisticData data =
      test_measure_linux_cpu->getStatisticsResults();
    ASSERT_TRUE(std::isnan(data.average));
    ASSERT_TRUE(std::isnan(data.min));
    ASSERT_TRUE(std::isnan(data.max));
    ASSERT_TRUE(std::isnan(data.standard_deviation));
    ASSERT_EQ(0, data.sample_count);
  }

  void TearDown() override
  {
    test_measure_linux_cpu->stop();
    test_measure_linux_cpu.reset();
    rclcpp::shutdown();
  }

protected:
  std::shared_ptr<TestLinuxCpuMeasurementNode> test_measure_linux_cpu;
};

TEST_F(LinuxCpuMeasurementTestFixture, testManualMeasurement) {
  // first measurement caches
  double cpu_active_percentage = test_measure_linux_cpu->periodicMeasurement();
  ASSERT_TRUE(std::isnan(cpu_active_percentage));
  // second measurement compares current and cached
  cpu_active_percentage = test_measure_linux_cpu->periodicMeasurement();
  ASSERT_DOUBLE_EQ(CPU_ACTIVE_PERCENTAGE, cpu_active_percentage);
}

TEST(LinuxCpuMeasurementTest, testParseProcLine)
{
  auto parsed_data = system_metrics_collector::processLine(proc_sample_1);

  ASSERT_EQ("cpu", parsed_data.cpu_label);
  ASSERT_EQ(22451232, parsed_data.times[0]);
  ASSERT_EQ(118653, parsed_data.times[1]);
  ASSERT_EQ(7348045, parsed_data.times[2]);
  ASSERT_EQ(934943300, parsed_data.times[3]);
  ASSERT_EQ(5378119, parsed_data.times[4]);
  ASSERT_EQ(0, parsed_data.times[5]);
  ASSERT_EQ(419114, parsed_data.times[6]);
  ASSERT_EQ(0, parsed_data.times[7]);

  ASSERT_EQ(
    "cpu_label=cpu, user=22451232, nice=118653, system=7348045, idle=934943300,"
    " iOWait=5378119, irq=0, softIrq=419114, steal=0",
    parsed_data.toString());
}

TEST(LinuxCpuMeasurementTest, testCalculateCpuActivePercentage)
{
  auto parsed_data1 = system_metrics_collector::processLine(proc_sample_1);
  auto parsed_data2 = system_metrics_collector::processLine(proc_sample_2);

  auto p = computeCpuActivePercentage(parsed_data1, parsed_data2);
  ASSERT_DOUBLE_EQ(CPU_ACTIVE_PERCENTAGE, p);
}

TEST(LinuxCpuMeasurementTest, testEmptyProcCpuData)
{
  system_metrics_collector::ProcCpuData empty;
  ASSERT_EQ(system_metrics_collector::ProcCpuData::EMPTY_LABEL, empty.cpu_label);

  for (int i = 0; i < static_cast<int>(system_metrics_collector::ProcCpuStates::kNumProcCpuStates);
    i++)
  {
    ASSERT_EQ(0, empty.times[i]);
  }
}
