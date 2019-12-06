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

#include "metrics_message_publisher.hpp"

#include <string>
#include <utility>

#include "metrics_statistics_msgs/msg/statistic_data_type.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;

namespace system_metrics_collector
{

MetricsMessage MetricsMessagePublisher::generateStatisticMessage(
  const std::string & node_name,
  const std::string & source_name,
  const builtin_interfaces::msg::Time window_start,
  const builtin_interfaces::msg::Time window_stop,
  const moving_average_statistics::StatisticData & data)
{
  MetricsMessage msg;

  msg.measurement_source_name = node_name;
  msg.metrics_source = source_name;
  msg.window_start = window_start;
  msg.window_stop = window_stop;

  msg.statistics.reserve(5);

  msg.statistics.emplace_back();
  msg.statistics.back().data_type = StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE;
  msg.statistics.back().data = data.average;

  msg.statistics.emplace_back();
  msg.statistics.back().data_type = StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM;
  msg.statistics.back().data = data.max;

  msg.statistics.emplace_back();
  msg.statistics.back().data_type = StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM;
  msg.statistics.back().data = data.min;

  msg.statistics.emplace_back();
  msg.statistics.back().data_type = StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT;
  msg.statistics.back().data = data.sample_count;

  msg.statistics.emplace_back();
  msg.statistics.back().data_type = StatisticDataType::STATISTICS_DATA_TYPE_STDDEV;
  msg.statistics.back().data = data.standard_deviation;

  return msg;
}

}  // namespace system_metrics_collector
