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

#ifndef MOVING_AVERAGE_STATISTICS__TYPES_HPP_
#define MOVING_AVERAGE_STATISTICS__TYPES_HPP_

#include <array>
#include <sstream>
#include <string>

#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

namespace moving_average_statistics
{

/**
 *  Enumeration of the various StatisticDataTypes
 */
constexpr const std::array<uint8_t, 5> STATISTICS_DATA_TYPES = {
  metrics_statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE,
  metrics_statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM,
  metrics_statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM,
  metrics_statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_STDDEV,
  metrics_statistics_msgs::msg::StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT
};

/**
 *  A container for statistics data results for a set of recorded observations.
 */
struct StatisticData
{
  double average = std::nan("");
  double min = std::nan("");
  double max = std::nan("");
  double standard_deviation = std::nan("");
  uint64_t sample_count = 0;
};

/**
 * Function which pretty prints the contents of a StatisticData struct.
 *
 * @param results the StatisticData to pretty print
 * @return std::string formatted struct contents in an easily readable format, e.g.,
 * /"avg=1, min=2, max=3, std_dev=4, count=5/"
 */
static std::string statisticsDataToString(const StatisticData & results)
{
  std::stringstream ss;
  ss << "avg=" << std::to_string(results.average) << ", min=" << std::to_string(results.min) <<
    ", max=" << std::to_string(results.max) << ", std_dev=" << std::to_string(
    results.standard_deviation) << ", count=" << std::to_string(results.sample_count);
  return ss.str();
}

}  // namespace moving_average_statistics

#endif  // MOVING_AVERAGE_STATISTICS__TYPES_HPP_
