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

#ifndef TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_
#define TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_

#include <chrono>
#include <mutex>
#include <string>

#include "topic_statistics_collector.hpp"
#include "system_metrics_collector/collector.hpp"

#include "rcl/time.h"
#include "rclcpp/clock.hpp"


namespace topic_statistics_collector
{

/**
 * Class used to measure the received messsage, tparam T, period from a ROS2 subscriber. This class
 * is thread safe and acquires a mutex when the member OnMessageReceived is executed.
 *
 * @tparam T the message type to receive from the subscriber / listener
*/
template<typename T>
class ReceivedMessagePeriodCollector : public TopicStatisticsCollector<T>
{
public:
  /**
   * Construct a ReceivedMessagePeriodCollector object. Set the
   * uninitialized_time_ member to use the input clock rcl_clock_type_t.
   * This is done because time can be compared iff they are provided by
   * the same clock type.
   *
   * @param clock input clock to use in order to measure received message
   * period, default is RCL_STEADY_TIME
   */
  explicit ReceivedMessagePeriodCollector(
    const rclcpp::Clock & clock = rclcpp::Clock{RCL_STEADY_TIME})
  : clock_{clock}
  {
    uninitialized_time_ = rclcpp::Time{0, 0, clock_.get_clock_type()};
    ResetTimeLastMessageReceived();
  }

  virtual ~ReceivedMessagePeriodCollector() = default;

  /**
   * Handle a message received and measure its received period. This member is thread safe and acquires
   * a lock to prevent race conditions when setting the time_last_message_received_ member.
   *
   * @param received_message
   */
  void OnMessageReceived(const T & received_message) override RCPPUTILS_TSA_REQUIRES(mutex_)
  {
    std::unique_lock<std::mutex> ulock{mutex_};
    const auto now = GetCurrentTime();

    if (time_last_message_received_ == uninitialized_time_) {
      time_last_message_received_ = now;
    } else {
      const std::chrono::nanoseconds nanos{now.nanoseconds() -
        time_last_message_received_.nanoseconds()};
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(nanos);
      time_last_message_received_ = now;
      system_metrics_collector::Collector::AcceptData(static_cast<double>(period.count()));
    }
  }

  /**
   * Return the current time using high_resolution_clock. Defined as virtual for testing
   * and if another clock implementation is desired.
   *
   * @return the current time provided by the clock given at construction time
   */
  virtual rclcpp::Time GetCurrentTime()
  {
    return clock_.now();
  }

protected:
  /**
   * Reset the time_last_message_received_ member.
   * @return true
   */
  bool SetupStart() override
  {
    ResetTimeLastMessageReceived();
    return true;
  }

  bool SetupStop() override
  {
    return true;
  }

private:
  /**
   * Resets time_last_message_received_ to the expected uninitialized_time_.
   */
  void ResetTimeLastMessageReceived()
  {
    time_last_message_received_ = uninitialized_time_;
  }

  /**
   * The clock to use in order to determine the rate OnMessageReceived is called.
   */
  rclcpp::Clock clock_;
  /**
   * Default uninitialized time. In order to compare rclcpp::Time they must come from
   * the same type of clock.
   */
  rclcpp::Time uninitialized_time_;
  rclcpp::Time time_last_message_received_ RCPPUTILS_TSA_GUARDED_BY(mutex_);
  mutable std::mutex mutex_;
};

}  // namespace topic_statistics_collector


#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_
