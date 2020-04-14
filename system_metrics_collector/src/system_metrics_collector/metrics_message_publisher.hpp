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
#include "statistics_msgs/msg/metrics_message.hpp"

#include "libstatistics_collector/moving_average_statistics/types.hpp"

namespace system_metrics_collector
{

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
};
}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__METRICS_MESSAGE_PUBLISHER_HPP_
