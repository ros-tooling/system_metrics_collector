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
#include <iostream>
#include <string>
#include <limits>
#include <memory>
#include <mutex>
#include <random>

#include "../../src/system_metrics_collector/utilities.hpp"
#include "test_constants.hpp"
#include "test_utilities.hpp"


TEST(UtilitiesTest, TestParseProcStatLine)
{
  auto parsed_data = system_metrics_collector::ProcessStatCpuLine(test_constants::kProcSamples[0]);

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
    parsed_data.ToString());
}

TEST(UtilitiesTest, TestParseProcStatLine2)
{
  auto parsed_data = system_metrics_collector::ProcessStatCpuLine(
    test_constants::kProcSampleResolutionTest);

  ASSERT_EQ("cpu", parsed_data.cpu_label);
  ASSERT_EQ(57211920, parsed_data.times[0]);
  ASSERT_EQ(335926, parsed_data.times[1]);
  ASSERT_EQ(18096939, parsed_data.times[2]);
  ASSERT_EQ(2526329830, parsed_data.times[3]);
  ASSERT_EQ(14818556, parsed_data.times[4]);
  ASSERT_EQ(0, parsed_data.times[5]);
  ASSERT_EQ(1072048, parsed_data.times[6]);
  ASSERT_EQ(0, parsed_data.times[7]);

  ASSERT_EQ(
    "cpu_label=cpu, user=57211920, nice=335926, system=18096939, idle=2526329830,"
    " iOWait=14818556, irq=0, softIrq=1072048, steal=0",
    parsed_data.ToString());
}

TEST(UtilitiesTest, TestMeasurePidCpuTime)
{
  auto parsed_data = system_metrics_collector::MeasurePidCpuTime();
  ASSERT_FALSE(parsed_data.IsMeasurementEmpty());
}

TEST(UtilitiesTest, TestEmptyProcCpuData)
{
  system_metrics_collector::ProcCpuData empty;

  ASSERT_EQ(system_metrics_collector::ProcCpuData::kEmptyLabel, empty.cpu_label);

  for (int i = 0; i < static_cast<int>(system_metrics_collector::ProcCpuStates::kNumProcCpuStates);
    i++)
  {
    ASSERT_EQ(0, empty.times[i]);
  }
}

TEST(UtilitiesTest, TestCalculateCpuActivePercentage)
{
  auto p = test_utilities::ComputeCpuActivePercentage(test_constants::kProcSamples[0],
      test_constants::kProcSamples[1]);
  ASSERT_DOUBLE_EQ(test_constants::kCpuActiveProcSample_0_1, p);
}

TEST(UtilitiesTest, TestCalculatePidCpuActivePercentage)
{
  using IntType = decltype(system_metrics_collector::ProcPidCpuData::total_cpu_time_);

  std::default_random_engine gen;
  std::uniform_int_distribution<IntType> dist{0, std::numeric_limits<IntType>::max() - 1};
  system_metrics_collector::ProcPidCpuData measurement1, measurement2;

  while (measurement1.total_cpu_time_ == measurement2.total_cpu_time_) {
    measurement1.pid_cpu_time_ = 100;
    measurement1.total_cpu_time_ = dist(gen);
    measurement2.pid_cpu_time_ = measurement1.pid_cpu_time_;
    measurement2.total_cpu_time_ = dist(gen);
  }

  auto p = system_metrics_collector::ComputePidCpuActivePercentage(measurement1, measurement2);

  ASSERT_DOUBLE_EQ(p, 0.0);
}

TEST(UtilitiesTest, TestProcMemInfoLines)
{
  auto d = system_metrics_collector::ProcessMemInfoLines(test_constants::kEmptySample);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::ProcessMemInfoLines(test_constants::kGarbageSample);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::ProcessMemInfoLines(test_constants::kIncompleteSample);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::ProcessMemInfoLines(test_constants::kCompleteSample);
  ASSERT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, d);

  d = system_metrics_collector::ProcessMemInfoLines(test_constants::kFullSample);
  ASSERT_DOUBLE_EQ(test_constants::kMemoryUsedPercentage, d);
}

TEST(UtilitiesTest, TestReadInvalidFile)
{
  const auto s = system_metrics_collector::ReadFileToString("this_will_fail.txt");
  ASSERT_EQ("", s);
}
