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

#ifndef SYSTEM_METRICS_COLLECTOR__METRICS_MESSAGE_PUBLISHER_HPP_
#define SYSTEM_METRICS_COLLECTOR__METRICS_MESSAGE_PUBLISHER_HPP_

#include <chrono>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "metrics_statistics_msgs/msg/metrics_message.hpp"

#include "../moving_average_statistics/types.hpp"

namespace system_metrics_collector
{

/**
 * Return a valid MetricsMessage ready to be published to a ROS topic
 *
 * @param node_name the name of the node that the data originates from
 * @param metric_name the name of the metric ("cpu_usage", "memory_usage", etc.)
 * @param unit name of the unit ("percentage", "mb", etc.)
 * @param window_start measurement window start time
 * @param window_stop measurement window end time
 * @param data statistics derived from the measurements made in the window
 * @return a MetricsMessage containing the statistics in the data parameter
 */
metrics_statistics_msgs::msg::MetricsMessage GenerateStatisticMessage(
        const std::string & node_name,
        const std::string & metric_name,
        const std::string & unit,
        const builtin_interfaces::msg::Time window_start,
        const builtin_interfaces::msg::Time window_stop,
        const moving_average_statistics::StatisticData & data
);

/**
 * Simple class to facilitate publishing messages containing statistics data
 */
class MetricsMessagePublisher
{
public:

  /**
   * Publish the statistics derived from the collected measurements (this is to be called via a
   * ROS2 timer per the publish_period)
   */
  virtual void PublishStatisticMessage() = 0;

  /**
   * Returns the name to use for this metric
   *
   * @return a string of the name for this measured metric
   */
  virtual std::string GetMetricName() const = 0;

  /**
   * Returns the name of the measurement unit of this metric
   *
   * @return a string of the name of the measurement unit of this metric
   */
  virtual const std::string & GetMetricUnit() const = 0;
};
}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__METRICS_MESSAGE_PUBLISHER_HPP_
