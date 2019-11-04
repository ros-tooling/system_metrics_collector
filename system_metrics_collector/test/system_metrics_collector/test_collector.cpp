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
#include <memory>
#include <iostream>
#include "system_metrics_collector/collector.hpp"
#include "moving_average_statistics/types.hpp"

/**
 * Simple extension to test basic functionality
 */
class TestCollector : public Collector<int> {
public:
  TestCollector() = default;
  virtual ~TestCollector() = default;
  bool start() override {
    return true;
  }
  bool stop() override {
    this->clearCurrentMeasurements();
    return true;
  }
};

/**
 * Test fixture
 */
class CollectorTestFixure : public ::testing::Test
{
public:
    void SetUp() override
    {
      test_collector = std::make_shared<TestCollector>();
    }

    void TearDown() override
    {
      test_collector.reset();
    }

protected:
    std::shared_ptr<TestCollector> test_collector = nullptr;
};

TEST_F(CollectorTestFixure, sanity) {
  ASSERT_TRUE(true);
  ASSERT_NE(test_collector, nullptr);
}

TEST_F(CollectorTestFixure, test_add_measurement) {

  test_collector->acceptData(1);
  auto stats = test_collector->getStatisticsResults();
  ASSERT_EQ(1, stats.sample_count);
  ASSERT_EQ(1, stats.average);
  std::cout << statisticsResultsToString(stats) << std::endl;
}

