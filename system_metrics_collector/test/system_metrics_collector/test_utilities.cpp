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

TEST(UtilitiesTest, testProcessLines)
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
