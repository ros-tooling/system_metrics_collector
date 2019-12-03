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

#include "batch_statistics_message_publisher.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;
using metrics_statistics_msgs::msg::StatisticDataPoint;
using metrics_statistics_msgs::msg::StatisticDataType;

namespace system_metrics_collector {

/* static */ constexpr const std::chrono::milliseconds BatchStatisticsMessagePublisher::
INVALID_PUBLISH_WINDOW;

MetricsMessage
BatchStatisticsMessagePublisher::generateStatisticMessage(
  std::string node_name,
  std::string source_name,
  builtin_interfaces::msg::Time window_start,
  builtin_interfaces::msg::Time window_end,
  const moving_average_statistics::StatisticData & data)
{
  MetricsMessage msg;

  msg.measurement_source_name = std::move(node_name);
  msg.metrics_source = std::move(source_name);
  msg.window_start = std::move(window_start);
  msg.window_stop = std::move(window_end);

  msg.statistics.resize(moving_average_statistics::STATISTICS_DATA_TYPES.size());
  msg.statistics[0].data_type = StatisticDataType::STATISTICS_DATA_TYPE_AVERAGE;
  msg.statistics[0].data = data.average;
  msg.statistics[1].data_type = StatisticDataType::STATISTICS_DATA_TYPE_MAXIMUM;
  msg.statistics[1].data = data.max;
  msg.statistics[2].data_type = StatisticDataType::STATISTICS_DATA_TYPE_MINIMUM;
  msg.statistics[2].data = data.min;
  msg.statistics[3].data_type = StatisticDataType::STATISTICS_DATA_TYPE_SAMPLE_COUNT;
  msg.statistics[3].data = data.sample_count;
  msg.statistics[4].data_type = StatisticDataType::STATISTICS_DATA_TYPE_STDDEV;
  msg.statistics[4].data = data.standard_deviation;

  return msg;
}

}  // namespace system_metrics_collector
