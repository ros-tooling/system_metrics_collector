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
#include <random>

#include "statistics_msgs/msg/statistic_data_type.hpp"
#include "rclcpp/rclcpp.hpp"

#include "system_metrics_collector/metrics_message_publisher.hpp"

#include "libstatistics_collector/collector/generate_statistics_message.hpp"

namespace
{
using libstatistics_collector::moving_average_statistics::StatisticData;
using statistics_msgs::msg::MetricsMessage;
using statistics_msgs::msg::StatisticDataPoint;
using statistics_msgs::msg::StatisticDataType;
using system_metrics_collector::MetricsMessagePublisher;

constexpr const char kTestNodeName[] = "test_publisher";
constexpr const char kTestMeasurementType[] = "test_measurement";
constexpr const char kTestMeasurementUnit[] = "test_unit";
}  // namespace


TEST(MetricsMessagePublisherTest, TestGenerateMessage) {
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>(kTestNodeName);
  rclcpp::Time time1 = node->now();
  rclcpp::Time time2 = node->now();

  std::default_random_engine gen;
  std::uniform_real_distribution<decltype(StatisticDataPoint::data)> dist(0.0, 100.0);
  StatisticData data;
  data.average = dist(gen);
  data.min = dist(gen);
  data.max = dist(gen);
  data.standard_deviation = dist(gen);
  data.sample_count = dist(gen);

  const auto msg = libstatistics_collector::collector::GenerateStatisticMessage(
    kTestNodeName, kTestMeasurementType, kTestMeasurementUnit, time1, time2, data);

  EXPECT_EQ(kTestNodeName, msg.measurement_source_name);
  EXPECT_EQ(kTestMeasurementType, msg.metrics_source);
  EXPECT_EQ(kTestMeasurementUnit, msg.unit);
  EXPECT_EQ(time1, msg.window_start);
  EXPECT_EQ(time2, msg.window_stop);

  for (const StatisticDataPoint & stat : msg.statistics) {
    switch (stat.data_type) {
      case StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE:
        EXPECT_FLOAT_EQ(data.average, stat.data);
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM:
        EXPECT_FLOAT_EQ(data.max, stat.data);
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM:
        EXPECT_FLOAT_EQ(data.min, stat.data);
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT:
        EXPECT_EQ(data.sample_count, static_cast<decltype(StatisticData::sample_count)>(stat.data));
        break;
      case StatisticDataType::STATISTICS_DATA_TYPE_STDDEV:
        EXPECT_FLOAT_EQ(data.standard_deviation, stat.data);
        break;
      default:
        ADD_FAILURE() << "Unexpected statistic data type";
        break;
    }
  }

  node.reset();
  rclcpp::shutdown();
}
