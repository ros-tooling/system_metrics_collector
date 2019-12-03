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

#ifndef SYSTEM_METRICS_COLLECTOR__BATCH_STATISTICS_MESSAGE_PUBLISHER_HPP_
#define SYSTEM_METRICS_COLLECTOR__BATCH_STATISTICS_MESSAGE_PUBLISHER_HPP_

#include <chrono>

#include "builtin_interfaces/msg/time.hpp"
#include "metrics_statistics_msgs/msg/metrics_message.hpp"

#include "../moving_average_statistics/types.hpp"

namespace system_metrics_collector {

class BatchStatisticsMessagePublisher
{
public:
  static constexpr const std::chrono::milliseconds INVALID_PUBLISH_WINDOW =
    std::chrono::milliseconds(0);

  static metrics_statistics_msgs::msg::MetricsMessage generateStatisticMessage(
    std::string node_name,
    std::string source_name,
    builtin_interfaces::msg::Time window_start,
    builtin_interfaces::msg::Time window_end,
    const moving_average_statistics::StatisticData & data
  );

  /**
   * Called via a ROS2 timer per the publish_period_. This publishes the statistics derived from
   * the collected measurements
   */
  virtual void publishStatisticMessage() = 0;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__COLLECTOR_HPP_
