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
#include "../../src/system_metrics_collector/linux_process_memory_measurement_node.hpp"

#include "test_constants.hpp"


namespace
{
constexpr const char TEST_STATM_LINE[] = "2084389 308110 7390 1 0 366785 0\n";
}

TEST(TestLinuxProcessMemoryMeasurement, testGetProcessUsedMemory) {
  auto ret = system_metrics_collector::getProcessUsedMemory(test_constants::GARBAGE_SAMPLE);
  EXPECT_TRUE(std::isnan(ret));
  ret = system_metrics_collector::getProcessUsedMemory(test_constants::EMPTY_SAMPLE);
  EXPECT_TRUE(std::isnan(ret));
  ret = system_metrics_collector::getProcessUsedMemory(TEST_STATM_LINE);
  EXPECT_EQ(2084389, ret);
}
