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
#include <string>

#include "topic_statistics_collector.hpp"

#include "../../src/system_metrics_collector/collector.hpp"

namespace topic_statistics_collector
{
constexpr const char kReceivedMessagePeriodMetricName[] = "received_message_period";
constexpr const char kUnits[] = "milliseconds";
constexpr const std::chrono::high_resolution_clock::time_point kDefaultTimePoint{std::chrono::
  high_resolution_clock::duration::zero()};

template<typename T>
class ReceivedMessagePeriod : public TopicStatisticsCollector<T>
{
public:
  /**
   * Constructs a ReceivedMessagePeriod object.
   *
   * This also starts the Collector.
   */
  ReceivedMessagePeriod()
  {
    system_metrics_collector::Collector::Start();
  }
  /**
   * Destructs a ReceivedMessagePeriod object.
   *
   * This also stops the Collector.
   */
  virtual ~ReceivedMessagePeriod()
  {
    system_metrics_collector::Collector::Stop();
  }

  /**
   * Handle a message received and measure its received period.
   * @param received_message
   */
  void OnMessageReceived(const T & received_message) override
  {
    auto now = GetCurrentTime();

    if (time_last_message_received_ == kDefaultTimePoint) {
      time_last_message_received_ = now;
    } else {
      int period = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - time_last_message_received_).count();
      time_last_message_received_ = now;
      system_metrics_collector::Collector::AcceptData(static_cast<double>(period));
    }
  }

protected:
  /**
   * Resets the time_last_message_received_ member.
   * @return
   */
  bool SetupStart() override
  {
    time_last_message_received_ = kDefaultTimePoint;
  }

  /**
   * No-op method
   * @return
   */
  bool SetupStop() override
  {
  }

  /**
   * Returns the current time using high_resolution_clock.
   * @return the current high_resolution_clock clock time
   */
  virtual std::chrono::high_resolution_clock::time_point GetCurrentTime()
  {
    return std::chrono::high_resolution_clock::now();
  }

private:
  std::chrono::high_resolution_clock::time_point time_last_message_received_{kDefaultTimePoint};
};

}  // namespace topic_statistics_collector


#endif  // TOPIC_STATISTICS_COLLECTOR__RECEIVED_MESSAGE_PERIOD_HPP_
