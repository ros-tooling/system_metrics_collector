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

#ifndef SYSTEM_METRICS_WORKSPACE__RECEIVED_MESSAGE_PERIOD_HPP_
#define SYSTEM_METRICS_WORKSPACE__RECEIVED_MESSAGE_PERIOD_HPP_

#include <string>
#include <chrono>
#include "topic_statistics_collector.hpp"

namespace system_metrics_collector
{

constexpr const char kReceivedMessagePeriodMetricName[] = "received_message_period";
constexpr const char kUnits[] = "milliseconds";
constexpr const std::chrono::high_resolution_clock::time_point kDefaultTimePoint{std::chrono::high_resolution_clock::duration::zero()};

template<typename T>
class ReceivedMessagePeriod : public system_metrics_collector::TopicStatisticsCollector<T>
{
public:

  //inherit from on message received
  ReceivedMessagePeriod() = default;
  virtual ~ReceivedMessagePeriod() = default;

  bool SetupStart()
  {
    time_last_message_received_ = kDefaultTimePoint;
  }
  //no op
  bool SetupStop() {}

  virtual double OnMessageReceived(const T & received_message) override
  {
    std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();

    if(time_last_message_received_ == kDefaultTimePoint) {
      time_last_message_received_ = now;
    } else {
      int period = std::chrono::duration_cast<std::chrono::milliseconds>(now - time_last_message_received_).count();
      double d = (double) period;
      Collector::AcceptData(d);
    }
  }

  std::chrono::high_resolution_clock::time_point time_last_message_received_;
};

}  // namespace system_metrics_collector


#endif //SYSTEM_METRICS_WORKSPACE_RECEIVEDMESSAGEPERIOD_HPP
