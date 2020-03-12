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

#ifndef TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_AGE_HPP_
#define TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_AGE_HPP_

#include <chrono>
#include <string>
#include <sstream>
#include <type_traits>

#include "topic_statistics_collector.hpp"

#include "rcl/time.h"
#include "rcutils/logging_macros.h"


namespace
{
template<typename M, typename = void>
struct HasHeader : public std::false_type {};

template<typename M>
struct HasHeader<M, decltype((void) M::header)>: std::true_type {};

template<typename M, typename Enable = void>
struct TimeStamp
{
  static uint64_t value(const M & m)
  {
    (void)m;
    return 0;
  }
};

template<typename M>
struct TimeStamp<M, typename std::enable_if<HasHeader<M>::value>::type>
{
  static uint64_t value(const M & m)
  {
    auto stamp = m.header.stamp;
    return stamp.nanosec;
  }
};
}  // namespace

namespace topic_statistics_collector
{
/**
 * Class used to measure the received messsage, tparam T, age from a ROS2 subscriber.
 *
 * @tparam T the message type to receive from the subscriber / listener
*/
template<typename T>
class ReceivedMessageAgeCollector : public TopicStatisticsCollector<T>
{
public:
  /**
   * Construct a ReceivedMessageAgeCollector object.
   *
   */
  ReceivedMessageAgeCollector()
  {}

  virtual ~ReceivedMessageAgeCollector() = default;

  /**
  * Handle a new incoming message. Calculate message age if a valid Header is present.
  *
  * @param received_message, the message to calculate age of.
  */
  void OnMessageReceived(const T & received_message, const uint64_t & now_nanoseconds) override
  {
    const auto timestamp_from_header = TimeStamp<T>::value(received_message);

    if (timestamp_from_header && now_nanoseconds) {
      const std::chrono::nanoseconds age_nanos{now_nanoseconds - timestamp_from_header};
      const auto age_millis = std::chrono::duration_cast<std::chrono::milliseconds>(age_nanos);

      system_metrics_collector::Collector::AcceptData(static_cast<double>(age_millis.count()));
    }
  }

protected:
  bool SetupStart() override
  {
    return true;
  }

  bool SetupStop() override
  {
    return true;
  }
};

}  // namespace topic_statistics_collector

#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_AGE_HPP_
