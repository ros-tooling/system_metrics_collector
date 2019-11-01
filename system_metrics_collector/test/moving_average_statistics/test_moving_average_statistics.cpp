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
#include <thread>
#include <atomic>
#include "moving_average_statistics/moving_average.hpp"


/**
 * Test fixture
 */
class MovingAverageStatisticsTestFixture : public ::testing::Test
{
public:
  void SetUp() override
  {
    moving_average_statistics = std::make_shared<MovingAverageStatistics<double>>();
  }

  void TearDown() override
  {
    moving_average_statistics->reset();
  }

protected:
  std::shared_ptr<MovingAverageStatistics<double>> moving_average_statistics = nullptr;
  int expected_count = 0;
};

// Useful testing constants
const double TEST_DATA[] = {-3.5, -2.1, -1.1, 0.0, 4.7, 5.0, 6.7, 9.9, 11.0};
const double EXPECTED_AVG = 3.4;
const double EXPECTED_MIN = -3.5;
const double EXPECTED_MAX = 11.0;
const double EXPECTED_STD = 4.997999599839919955173;
const double EXPECTED_SIZE = 9;

TEST_F(MovingAverageStatisticsTestFixture, sanity) {
  ASSERT_TRUE(true);
  ASSERT_NE(moving_average_statistics, nullptr);
}

TEST_F(MovingAverageStatisticsTestFixture, test_average) {
  for (double d : TEST_DATA) {
    moving_average_statistics->add_item(d);
    ASSERT_EQ(++expected_count, moving_average_statistics->get_count());
  }

  EXPECT_DOUBLE_EQ(moving_average_statistics->average(), EXPECTED_AVG);
}

TEST_F(MovingAverageStatisticsTestFixture, test_average_empty) {
  ASSERT_TRUE(std::isnan(moving_average_statistics->average()));
}

TEST_F(MovingAverageStatisticsTestFixture, test_maximum) {
  for (double d : TEST_DATA) {
    moving_average_statistics->add_item(d);
    ASSERT_EQ(++expected_count, moving_average_statistics->get_count());
  }

  EXPECT_EQ(moving_average_statistics->max(), EXPECTED_MAX);
}

TEST_F(MovingAverageStatisticsTestFixture, test_maximum_empty) {
  ASSERT_TRUE(std::isnan(moving_average_statistics->max()));
}

TEST_F(MovingAverageStatisticsTestFixture, test_minimum) {
  for (double d : TEST_DATA) {
    moving_average_statistics->add_item(d);
    ASSERT_EQ(++expected_count, moving_average_statistics->get_count());
  }

  const double expected_minimum = -3.5;
  EXPECT_EQ(moving_average_statistics->min(), expected_minimum);
}

TEST_F(MovingAverageStatisticsTestFixture, test_minimum_empty) {
  ASSERT_TRUE(std::isnan(moving_average_statistics->min()));
}

TEST_F(MovingAverageStatisticsTestFixture, test_standard_deviation) {
  for (double d : TEST_DATA) {
    moving_average_statistics->add_item(d);
  }

  EXPECT_DOUBLE_EQ(moving_average_statistics->standardDeviation(), EXPECTED_STD);
}

TEST_F(MovingAverageStatisticsTestFixture, test_get_statistics) {
  for (double d : TEST_DATA) {
    moving_average_statistics->add_item(d);
  }

  auto result = moving_average_statistics->getStatistics();
  EXPECT_DOUBLE_EQ(result.average, EXPECTED_AVG);
  EXPECT_DOUBLE_EQ(result.min, EXPECTED_MIN);
  EXPECT_DOUBLE_EQ(result.max, EXPECTED_MAX);
  EXPECT_DOUBLE_EQ(result.standard_deviation, EXPECTED_STD);
  EXPECT_DOUBLE_EQ(result.sample_count, EXPECTED_SIZE);
}

TEST_F(MovingAverageStatisticsTestFixture, test_get_statistics_int) {
  auto data_int = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

  const double expected_average = 5.5;
  const double expected_minimum = 1;
  const double expected_maximum = 10;
  const double expected_std = 2.8722813232690143;
  const int expected_size = 10;

  for (int d : data_int) {
    moving_average_statistics->add_item(d);
  }

  auto result = moving_average_statistics->getStatistics();
  EXPECT_DOUBLE_EQ(result.average, expected_average);
  EXPECT_DOUBLE_EQ(result.min, expected_minimum);
  EXPECT_DOUBLE_EQ(result.max, expected_maximum);
  EXPECT_DOUBLE_EQ(result.standard_deviation, expected_std);
  EXPECT_DOUBLE_EQ(result.sample_count, expected_size);
}

TEST_F(MovingAverageStatisticsTestFixture, test_standard_deviation_empty) {
  ASSERT_TRUE(std::isnan(moving_average_statistics->standardDeviation()));
}

TEST_F(MovingAverageStatisticsTestFixture, test_reset) {
  moving_average_statistics->add_item(0.6);
  moving_average_statistics->reset();
  ASSERT_TRUE(std::isnan(moving_average_statistics->average()));
  moving_average_statistics->add_item(1.5);
  EXPECT_EQ(moving_average_statistics->average(), 1.5);
}

TEST_F(MovingAverageStatisticsTestFixture, test_thread_safe) {
  std::atomic<int> total_sum(0);
  std::atomic<int> count(0);

  std::thread t1([this, &count, &total_sum]() {
      for (double i = 1; i < 1101.0; i++) {
        moving_average_statistics->add_item(i);
        count++;
        total_sum += i;
      }
    });
  std::thread t2([this, &count, &total_sum]() {
      for (double i = 1; i < 2101.0; i++) {
        moving_average_statistics->add_item(i);
        count++;
        total_sum += i;
      }
    });
  std::thread t3([this, &count, &total_sum]() {
      for (double i = 1; i < 3101.0; i++) {
        moving_average_statistics->add_item(i);
        count++;
        total_sum += i;
      }
    });

  t1.join();
  t2.join();
  t3.join();

  double control = static_cast<double>(total_sum.load()) / static_cast<double>(count.load());
  double var = 0.00000000001;

  ASSERT_NEAR(moving_average_statistics->average(), control, var);
}

TEST_F(MovingAverageStatisticsTestFixture, asdf) {
  auto test = std::make_shared<MovingAverageStatistics<int>>();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // run with 'time ./test_moving_average_statistics --gtest_repeat=2000
  //   --gtest_shuffle --gtest_break_on_failure'
  return RUN_ALL_TESTS();
}
