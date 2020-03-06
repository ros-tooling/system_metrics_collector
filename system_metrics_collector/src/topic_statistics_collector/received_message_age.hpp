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

#include "topic_statistics_collector.hpp"

#include "message_filters/message_traits.h"
#include "rcl/time.h"
#include "rclcpp/clock.hpp"
#include "rcutils/logging_macros.h"


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
   * @param clock input clock to use to calculate received message age.
   * Default is RCL_STEADY_TIME
   */
  explicit ReceivedMessageAgeCollector(
    const rclcpp::Clock & clock = rclcpp::Clock{RCL_STEADY_TIME})
  : clock_{clock}
  {}

  virtual ~ReceivedMessageAgeCollector() = default;

  /**
  * Handle a new incoming message. Calculate message age if a valid Header is present.
  *
  * @param received_message, the message to calculate age of.
  */
  void OnMessageReceived(const T & received_message) override
  {
    const auto timestamp_from_header = message_filters::message_traits::TimeStamp<T>::value(
      received_message);

    if (timestamp_from_header.nanoseconds()) {
      const auto now = GetCurrentTime();

      if (timestamp_from_header.get_clock_type() == now.get_clock_type()) {
        const std::chrono::nanoseconds age_nanos{
          now.nanoseconds() - timestamp_from_header.nanoseconds()};
        const auto age_millis = std::chrono::duration_cast<std::chrono::milliseconds>(age_nanos);

        system_metrics_collector::Collector::AcceptData(static_cast<double>(age_millis.count()));
      } else {
        std::stringstream warn_msg;
        warn_msg <<
          "Message header and current clock have different time sources, " <<
          "cannot measure message age" <<
          "\nmessage header clock type: " << timestamp_from_header.get_clock_type() <<
          "\ncollector clock type: " << clock_.get_clock_type();
        RCUTILS_LOG_WARN_NAMED("topic_statistics_collector", "%s", warn_msg.str().c_str());
      }
    }
  }

  /**
   * Return the current time using high_resolution_clock.
   * Defined as virtual for testing and if another clock implementation is desired.
   *
   * @return the current time provided by the clock given at construction time
   */
  virtual rclcpp::Time GetCurrentTime()
  {
    return clock_.now();
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

private:
  /**
   * The clock to use in order to determine the age of incoming messages.
   */
  rclcpp::Clock clock_;
};

}  // namespace topic_statistics_collector

#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_AGE_HPP_
