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


#ifndef SYSTEM_METRICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_
#define SYSTEM_METRICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_

#include <chrono>
#include <string>

#include "../system_metrics_collector/collector.hpp"

namespace system_metrics_collector
{
/**
 * Class template to collect topic statistics for ROS 2 topics.
 */

//todo rename to have subscriber in the name
template<typename T>
class TopicStatisticsCollector : public system_metrics_collector::Collector
{
public:
  /**
  * Constructs a TopicStatisticsCollector object.
  */
  TopicStatisticsCollector() = default;

  virtual ~TopicStatisticsCollector() = default;

private:
  /**
   * Handle receiving a single message on subscribed topic.
   * This is called by the subscriber that listens to topic for which statistics are to be measured.
   *
   * @return the measurement made to be aggregated for statistics
   */
  virtual double OnMessageReceived(const T & received_message) = 0;
};

}  // namespace system_metrics_collector

#endif  // TOPIC_STATISTICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_
