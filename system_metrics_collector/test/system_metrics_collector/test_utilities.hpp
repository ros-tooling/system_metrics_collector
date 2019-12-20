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

#ifndef SYSTEM_METRICS_COLLECTOR__TEST_UTILITIES_HPP_
#define SYSTEM_METRICS_COLLECTOR__TEST_UTILITIES_HPP_

#include <string>

#include "../../src/system_metrics_collector/utilities.hpp"

/**
 * Constants used and shared among the various system metrics collector tests.
 */
namespace test_utilities
{

inline double computeCpuActivePercentage(const std::string & data1, const std::string & data2)
{
  auto parsed_data1 = system_metrics_collector::processStatCpuLine(data1);
  auto parsed_data2 = system_metrics_collector::processStatCpuLine(data2);
  return system_metrics_collector::computeCpuActivePercentage(parsed_data1, parsed_data2);
}

inline double computePidCpuActivePercentage(
  const std::string & pid_data1,
  const std::string & sys_data1,
  const std::string & pid_data2,
  const std::string & sys_data2)
{
  auto parsed_pid_data1 = system_metrics_collector::processPidStatCpuLine(pid_data1);
  auto parsed_pid_data2 = system_metrics_collector::processPidStatCpuLine(pid_data2);
  auto parsed_sys_data1 = system_metrics_collector::processStatCpuLine(sys_data1);
  auto parsed_sys_data2 = system_metrics_collector::processStatCpuLine(sys_data2);
  return system_metrics_collector::computePidCpuActivePercentage(
    parsed_pid_data1,
    parsed_sys_data1,
    parsed_pid_data2,
    parsed_sys_data2);
}

}  // namespace test_utilities

#endif  // SYSTEM_METRICS_COLLECTOR__TEST_UTILITIES_HPP_
