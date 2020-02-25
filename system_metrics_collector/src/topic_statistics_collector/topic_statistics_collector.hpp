// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


#ifndef TOPIC_STATISTICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_
#define TOPIC_STATISTICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_

#include <chrono>
#include <string>

#include "../system_metrics_collector/collector.hpp"

namespace topic_statistics_collector
{
/**
 * Interface to collect and perform measurements for ROS2 topic statistics.
 *
 * @tparam T the ROS2 message type to collect
 */
template<typename T>
class TopicStatisticsCollector : public system_metrics_collector::Collector
{
public:
  TopicStatisticsCollector() = default;
  virtual ~TopicStatisticsCollector() = default;

private:
  /**
   * Handle receiving a single message of type T.
   * @tparam T the ROS2 message type to collect
   */
  virtual void OnMessageReceived(const T & received_message) = 0;
};

}  // namespace topic_statistics_collector

#endif  // TOPIC_STATISTICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_
