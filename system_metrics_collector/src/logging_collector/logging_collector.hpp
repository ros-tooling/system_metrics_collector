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

#ifndef SYSTEM_METRICS_COLLECTOR__LOGGING_COLLECTOR_HPP_
#define SYSTEM_METRICS_COLLECTOR__LOGGING_COLLECTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>
#include <iostream>
#include <rcl_interfaces/msg/log.hpp>

#include "../moving_average_statistics/moving_average.hpp"
#include "../moving_average_statistics/types.hpp"

#include "../system_metrics_collector/metrics_message_publisher.hpp"
#include "../system_metrics_collector/constants.hpp"

namespace logging_collector {

using std::placeholders::_1;
using namespace moving_average_statistics;
using namespace system_metrics_collector;

class LoggingCollector : public rclcpp::Node {

public:
    LoggingCollector()
    : Node("test")
    {
      window_start_= now();
      subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
              "rosout", 10, std::bind(&LoggingCollector::topic_callback, this, _1));

      publish_timer_ = this->create_wall_timer(
              std::chrono::seconds{20}, [this]() {this->print_stats();});

      collection_timer_ = this->create_wall_timer(
              std::chrono::seconds{4}, [this]() {this->collect();});
      publisher_ = create_publisher<metrics_statistics_msgs::msg::MetricsMessage>(
              collector_node_constants::kStatisticsTopicName,
              10 /*history_depth*/);
    }

private:

    void collect()
    {
      std::cout << "Collecting" << std::endl;

      debug_collector_.AddMeasurement( (double) debug_counter_.load());
      info_collector_.AddMeasurement( (double) info_counter_.load());
      warn_collector_.AddMeasurement( (double) warn_counter_.load());
      error_collector_.AddMeasurement( (double) error_counter_.load());
      fatal_collector_.AddMeasurement( (double) fatal_counter_.load());

    }

    void print_stats()
    {
      std::cout << "=============================" << std::endl;
      std::cout << "DEBUG stats: " << StatisticsDataToString(debug_collector_.GetStatistics()) << std::endl;
      std::cout << "INFO stats: " << StatisticsDataToString(info_collector_.GetStatistics()) << std::endl;
      std::cout << "WARN stats: " << StatisticsDataToString(warn_collector_.GetStatistics()) << std::endl;
      std::cout << "ERROR stats: " << StatisticsDataToString(error_collector_.GetStatistics()) << std::endl;
      std::cout << "FATAL stats: " << StatisticsDataToString(fatal_collector_.GetStatistics()) << std::endl;
      std::cout << "=============================" << std::endl;

      auto now = this->now();

      //publish it
      auto message1 = GenerateStatisticMessage("log_collector",
                                              "DEBUG_COUNT",
                                              "count",
                                              window_start_,
                                              now,
                                              debug_collector_.GetStatistics());
      auto message2 = GenerateStatisticMessage("log_collector",
                                              "DEBUG_COUNT",
                                               "count",
                                              window_start_,
                                              now,
                                              debug_collector_.GetStatistics());
      auto message3 = GenerateStatisticMessage("log_collector",
                                              "WARN_COUNT",
                                               "count",
                                              window_start_,
                                              now,
                                              info_collector_.GetStatistics());
      auto message4 = GenerateStatisticMessage("log_collector",
                                              "ERROR_COUNT",
                                               "count",
                                              window_start_,
                                              now,
                                              error_collector_.GetStatistics());
      auto message5 = GenerateStatisticMessage("log_collector",
                                              "FATAL_COUNT",
                                               "count",
                                              window_start_,
                                              now,
                                              fatal_collector_.GetStatistics());

      publisher_->publish(message1);
      publisher_->publish(message2);
      publisher_->publish(message3);
      publisher_->publish(message4);
      publisher_->publish(message5);

      window_start_ = this->now();
    }

    void topic_callback(const rcl_interfaces::msg::Log::SharedPtr log_msg)
    {
      switch(log_msg->level) {
        case rcl_interfaces::msg::Log::DEBUG:
          ++debug_counter_;
          break;
        case rcl_interfaces::msg::Log::INFO:
          ++info_counter_;
          break;
        case rcl_interfaces::msg::Log::WARN:
          ++warn_counter_;
          break;
        case rcl_interfaces::msg::Log::ERROR:
          ++error_counter_;
          break;
        case rcl_interfaces::msg::Log::FATAL:
          ++fatal_counter_;
          break;
        default:
          std::cout << "unsupported levelL " << log_msg->level << std::endl;
      }
    }
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
    rclcpp::Publisher<metrics_statistics_msgs::msg::MetricsMessage>::SharedPtr publisher_;

    std::atomic<std::uint64_t> debug_counter_;
    std::atomic<std::uint64_t> info_counter_;
    std::atomic<std::uint64_t> warn_counter_;
    std::atomic<std::uint64_t> error_counter_;
    std::atomic<std::uint64_t> fatal_counter_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr collection_timer_;

    MovingAverageStatistics debug_collector_;
    MovingAverageStatistics info_collector_;
    MovingAverageStatistics error_collector_;
    MovingAverageStatistics warn_collector_;
    MovingAverageStatistics fatal_collector_;
    rclcpp::Time window_start_;
};


} // namespace logging_collector
#endif //SYSTEM_METRICS_COLLECTOR__LOGGING_COLLECTOR_HPP_
