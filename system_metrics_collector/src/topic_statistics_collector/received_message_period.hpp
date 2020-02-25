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

#include "../../src/system_metrics_collector/collector.hpp"

namespace topic_statistics_collector
{
constexpr const std::chrono::steady_clock::time_point kDefaultTimePoint{std::chrono::
  steady_clock::duration::zero()};

/**
 * Class used to measure the received messsage, tparam T, period from a ROS2 subscriber. This class
 * is thread safe and acquires a mutex when the member OnMessageReceived is executed.
 * @tparam T the message to receive from the subscriber / listener
 */
template<typename T>
class ReceivedMessagePeriodCollector : public TopicStatisticsCollector<T>
{
public:
  /**
   * Construct a ReceivedMessagePeriod object.
   *
   * This also starts the Collector.
   */
  ReceivedMessagePeriodCollector() = default;
  /**
   * Destruct a ReceivedMessagePeriod object.
   *
   * This also stops the Collector.
   */
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

    if (time_last_message_received_ == kDefaultTimePoint) {
      time_last_message_received_ = now;
    } else {
      const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - time_last_message_received_).count();
      time_last_message_received_ = now;
      system_metrics_collector::Collector::AcceptData(static_cast<double>(period));
    }
  }

protected:
  /**
   * Reset the time_last_message_received_ member.
   * @return
   */
  bool SetupStart() override
  {
    time_last_message_received_ = kDefaultTimePoint;
    return true;
  }

  bool SetupStop() override
  {
    return true;
  }

  /**
   * Return the current time using high_resolution_clock. Defined as virtual for testing
   * and if another clock implementation is desired.
   * @return the current high_resolution_clock clock time
   */
  virtual std::chrono::steady_clock::time_point GetCurrentTime() const
  {
    return std::chrono::steady_clock::now();
  }

private:
  std::chrono::steady_clock::time_point time_last_message_received_{kDefaultTimePoint}
  RCPPUTILS_TSA_GUARDED_BY(mutex_);

  mutable std::mutex mutex_;
};

}  // namespace topic_statistics_collector


#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_
