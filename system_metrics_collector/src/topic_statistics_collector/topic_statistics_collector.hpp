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

#include <string>

#include "../system_metrics_collector/collector.hpp"

namespace topic_statistics_collector
{
/**
 * Class template to collect topic statistics for ROS 2 topics.
 */
template<typename T>
class TopicStatisticsCollector : public system_metrics_collector::Collector
{
public:
  /**
  * Constructs a TopicStatisticsCollector object.
  */
  TopicStatisticsCollector() = default;

  virtual ~TopicStatisticsCollector() = default;

  /**
   * Returns a pretty printed status representation of this class.
   *
   * @return a string detailing the current status
   */
  std::string GetStatusString() const override = 0;

protected:
  /**
   * No-op setup.
   */
  bool SetupStart() override = 0;

  /**
   * No-op teardown.
   */
  bool SetupStop() override = 0;

private:
  /**
   * Handle receiving a single message on subscribed topic.
   * This is called by the subscriber that listens to topic for which statistics are to be measured.
   *
   * @return the measurement made to be aggregated for statistics
   */
  virtual double OnMessageReceived(
    const typename T::SharedPtr,
    const std::chrono::time_point & now) = 0;
};

}  // namespace topic_statistics_collector

#endif  // TOPIC_STATISTICS_COLLECTOR__TOPIC_STATISTICS_COLLECTOR_HPP_
