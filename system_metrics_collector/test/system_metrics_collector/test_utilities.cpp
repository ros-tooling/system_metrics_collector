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
#include <memory>
#include <mutex>

#include "../../src/system_metrics_collector/utilities.hpp"
#include "test_constants.hpp"
#include "test_utilities.hpp"


TEST(UtilitiesTest, testParseProcStatLine)
{
  auto parsed_data = system_metrics_collector::processStatCpuLine(test_constants::PROC_SAMPLES[0]);

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

TEST(UtilitiesTest, testParseProcStatLine2)
{
  auto parsed_data = system_metrics_collector::processStatCpuLine(
    test_constants::PROC_SAMPLE_RESOLUTION_TEST);

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
    parsed_data.toString());
}

TEST(UtilitiesTest, testParseProcPidStatLine)
{
  auto parsed_data = system_metrics_collector::processPidStatCpuLine(test_constants::EMPTY_SAMPLE);
  ASSERT_TRUE(parsed_data.isMeasurementEmpty());

  parsed_data = system_metrics_collector::processPidStatCpuLine(test_constants::GARBAGE_SAMPLE);
  ASSERT_TRUE(parsed_data.isMeasurementEmpty());

  parsed_data = system_metrics_collector::processPidStatCpuLine(
    test_constants::PROC_PID_SAMPLES[0]);
  ASSERT_EQ(20701, parsed_data.utime);
  ASSERT_EQ(2305, parsed_data.stime);
  ASSERT_EQ("utime=20701, stime=2305", parsed_data.toString());
}

TEST(UtilitiesTest, testEmptyProcCpuData)
{
  system_metrics_collector::ProcCpuData empty;

  ASSERT_EQ(system_metrics_collector::ProcCpuData::EMPTY_LABEL, empty.cpu_label);

  for (int i = 0; i < static_cast<int>(system_metrics_collector::ProcCpuStates::kNumProcCpuStates);
    i++)
  {
    ASSERT_EQ(0, empty.times[i]);
  }
}

TEST(UtilitiesTest, testCalculateCpuActivePercentage)
{
  auto p = test_utilities::computeCpuActivePercentage(test_constants::PROC_SAMPLES[0],
      test_constants::PROC_SAMPLES[1]);
  ASSERT_DOUBLE_EQ(test_constants::CPU_ACTIVE_PROC_SAMPLE_0_1, p);
}

TEST(UtilitiesTest, testCalculatePidCpuActivePercentage)
{
  auto p = test_utilities::computePidCpuActivePercentage(
    test_constants::PROC_PID_SAMPLES[0],
    test_constants::PROC_SAMPLES[0],
    test_constants::PROC_PID_SAMPLES[1],
    test_constants::PROC_SAMPLES[1]);
  ASSERT_DOUBLE_EQ(test_constants::CPU_ACTIVE_PROC_PID_SAMPLE_0_1, p);
}

TEST(UtilitiesTest, testProcMemInfoLines)
{
  auto d = system_metrics_collector::processMemInfoLines(test_constants::EMPTY_SAMPLE);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::processMemInfoLines(test_constants::GARBAGE_SAMPLE);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::processMemInfoLines(test_constants::INCOMPLETE_SAMPLE);
  ASSERT_TRUE(std::isnan(d));

  d = system_metrics_collector::processMemInfoLines(test_constants::COMPLETE_SAMPLE);
  ASSERT_DOUBLE_EQ(test_constants::MEMORY_USED_PERCENTAGE, d);

  d = system_metrics_collector::processMemInfoLines(test_constants::FULL_SAMPLE);
  ASSERT_DOUBLE_EQ(test_constants::MEMORY_USED_PERCENTAGE, d);
}

TEST(UtilitiesTest, testReadInvalidFile)
{
  const auto s = system_metrics_collector::readFileToString("this_will_fail.txt");
  ASSERT_EQ("", s);
}
