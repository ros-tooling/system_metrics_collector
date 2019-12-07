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

#include <iostream>
#include <memory>

#include "../../src/system_metrics_collector/collector.hpp"
#include "../../src/moving_average_statistics/types.hpp"

/**
 * Simple extension to test basic functionality
 */
class TestCollector : public system_metrics_collector::Collector
{
public:
  TestCollector() = default;
  virtual ~TestCollector() = default;
  bool setupStart() override
  {
    return true;
  }
  bool setupStop() override
  {
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
    test_collector = std::make_unique<TestCollector>();
    ASSERT_FALSE(test_collector->isStarted());
  }

  void TearDown() override
  {
    test_collector->stop();  // don't assert as tests can call stop
    ASSERT_FALSE(test_collector->isStarted());
    test_collector.reset();
  }

protected:
  std::unique_ptr<TestCollector> test_collector{};
};

TEST_F(CollectorTestFixure, sanity) {
  ASSERT_NE(test_collector, nullptr);
}

TEST_F(CollectorTestFixure, test_add_and_clear_measurement) {
  test_collector->acceptData(1);
  auto stats = test_collector->getStatisticsResults();
  ASSERT_EQ(1, stats.sample_count);
  ASSERT_EQ(1, stats.average);

  test_collector->clearCurrentMeasurements();

  stats = test_collector->getStatisticsResults();
  ASSERT_TRUE(std::isnan(stats.average));
  ASSERT_TRUE(std::isnan(stats.min));
  ASSERT_TRUE(std::isnan(stats.max));
  ASSERT_TRUE(std::isnan(stats.standard_deviation));
  ASSERT_EQ(0, stats.sample_count);
}

TEST_F(CollectorTestFixure, test_start_and_stop) {
  ASSERT_FALSE(test_collector->isStarted());
  ASSERT_EQ("started=false, avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_collector->getStatusString());

  ASSERT_TRUE(test_collector->start());
  ASSERT_TRUE(test_collector->isStarted());
  ASSERT_EQ("started=true, avg=nan, min=nan, max=nan, std_dev=nan, count=0",
    test_collector->getStatusString());

  ASSERT_TRUE(test_collector->stop());
  ASSERT_FALSE(test_collector->isStarted());
}
