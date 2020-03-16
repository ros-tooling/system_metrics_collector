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


namespace topic_statistics_collector
{

static constexpr const int64_t kUninitializedTime{0};

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
   * Construct a ReceivedMessagePeriodCollector object.
   *
   */
  ReceivedMessagePeriodCollector()
  {
    ResetTimeLastMessageReceived();
  }

  virtual ~ReceivedMessagePeriodCollector() = default;

  /**
   * Handle a message received and measure its received period. This member is thread safe and acquires
   * a lock to prevent race conditions when setting the time_last_message_received_ member.
   *
   * @param received_message
   * @param time the message was received in nanoseconds
   */
  void OnMessageReceived(const T & received_message, const rcl_time_point_value_t now_nanoseconds)
  override RCPPUTILS_TSA_REQUIRES(mutex_)
  {
    std::unique_lock<std::mutex> ulock{mutex_};

    if (time_last_message_received_ == kUninitializedTime) {
      time_last_message_received_ = now_nanoseconds;
    } else {
      const std::chrono::nanoseconds nanos{now_nanoseconds - time_last_message_received_};
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(nanos);
      time_last_message_received_ = now_nanoseconds;
      system_metrics_collector::Collector::AcceptData(static_cast<double>(period.count()));
    }
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
    time_last_message_received_ = kUninitializedTime;
  }

  /**
   * Default uninitialized time.
   */
  rcl_time_point_value_t time_last_message_received_{kUninitializedTime}
  RCPPUTILS_TSA_GUARDED_BY(mutex_);
  mutable std::mutex mutex_;
};

}  // namespace topic_statistics_collector


#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_
