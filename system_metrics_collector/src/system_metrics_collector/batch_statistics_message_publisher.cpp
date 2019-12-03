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

namespace system_metrics_collector
{

/* static */ constexpr const std::chrono::milliseconds BatchStatisticsMessagePublisher::
INVALID_PUBLISH_WINDOW;

MetricsMessage
BatchStatisticsMessagePublisher::generateStatisticMessage()
{
  MetricsMessage msg;

  // msg.measurement_source_name = get_name();
  // msg.window_start = measurement_start_;
  // msg.window_stop = now();
  // return msg;

  // const moving_average_statistics::StatisticData statistic_data = getStatisticsResults();
  // const double data[moving_average_statistics::STATISTICS_DATA_TYPES.size()] = {
  //   statistic_data.average,
  //   statistic_data.min,
  //   statistic_data.max,
  //   statistic_data.standard_deviation,
  //   static_cast<double>(statistic_data.sample_count)
  // };

  // MetricsMessage msg = newMetricsMessage();
  // msg.metrics_source = "memory_usage";
  // for (int i = 0; i < moving_average_statistics::STATISTICS_DATA_TYPES.size(); ++i) {
  //   msg.statistics.emplace_back();
  //   msg.statistics.back().data_type = moving_average_statistics::STATISTICS_DATA_TYPES[i];
  //   msg.statistics.back().data = data[i];
  // }

  return msg;
}

}  // namespace system_metrics_collector
