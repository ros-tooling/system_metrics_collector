// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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


#include "periodic_measurement_node.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>

#include "constants.hpp"
#include "rclcpp/rclcpp.hpp"

using metrics_statistics_msgs::msg::MetricsMessage;

namespace system_metrics_collector
{

PeriodicMeasurementNode::PeriodicMeasurementNode(
  const std::string & name,
  const rclcpp::NodeOptions & options)
: Node{name, options}
{
  rcl_interfaces::msg::IntegerRange positive_range;
  positive_range.from_value = 1;
  positive_range.to_value =
    std::numeric_limits<decltype(rcl_interfaces::msg::IntegerRange::to_value)>::max();
  positive_range.step = 1;

  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.read_only = true;
  descriptor.integer_range.push_back(positive_range);

  // rclcpp::Node throws if name is empty

  descriptor.description = "The period in milliseconds between each measurement";
  auto measurement_period = declare_parameter(
    collector_node_constants::kCollectPeriodParam,
    collector_node_constants::kDefaultCollectPeriod.count(),
    descriptor);
  measurement_period_ = std::chrono::milliseconds{measurement_period};

  descriptor.description = "The period in milliseconds between each published MetricsMessage";
  auto publish_period = declare_parameter(
    collector_node_constants::kPublishPeriodParam,
    collector_node_constants::kDefaultPublishPeriod.count(),
    descriptor);
  publish_period_ = std::chrono::milliseconds{publish_period};

  if (publish_period_ <= measurement_period_) {
    throw std::invalid_argument{
            "publish_period cannot be less than or equal to the measurement_period"};
  }
}

bool PeriodicMeasurementNode::SetupStart()
{
  assert(measurement_timer_ == nullptr);
  assert(publish_timer_ == nullptr);

  RCLCPP_DEBUG(this->get_logger(), "SetupStart: creating measurement_timer_");
  measurement_timer_ = this->create_wall_timer(
    measurement_period_, [this]() {this->PerformPeriodicMeasurement();});

  if (publisher_ == nullptr) {
    publisher_ = create_publisher<MetricsMessage>(collector_node_constants::kStatisticsTopicName,
        10 /*history_depth*/);
  }

  RCLCPP_DEBUG(this->get_logger(), "SetupStart: creating publish_timer_");
  publish_timer_ = this->create_wall_timer(
    publish_period_, [this]() {
      this->PublishStatisticMessage();
      this->ClearCurrentMeasurements();
      this->window_start_ = this->now();
    });

  window_start_ = now();

  return true;
}

bool PeriodicMeasurementNode::SetupStop()
{
  assert(measurement_timer_ != nullptr);
  assert(publish_timer_ != nullptr);
  assert(publisher_ != nullptr);

  measurement_timer_->cancel();
  measurement_timer_.reset();

  publish_timer_->cancel();
  publish_timer_.reset();

  return true;
}

std::string PeriodicMeasurementNode::GetStatusString() const
{
  std::stringstream ss;
  ss << "name=" << get_name() <<
    ", measurement_period=" << std::to_string(measurement_period_.count()) << "ms" <<
    ", publishing_topic=" << (publisher_ ? publisher_->get_topic_name() : "") <<
    ", publish_period=" << std::to_string(publish_period_.count()) + "ms" <<
    ", " << Collector::GetStatusString();
  return ss.str();
}

void PeriodicMeasurementNode::PerformPeriodicMeasurement()
{
  const double measurement = PeriodicMeasurement();
  RCLCPP_DEBUG(this->get_logger(), "PerformPeriodicMeasurement: %f", measurement);

  AcceptData(measurement);
  RCLCPP_DEBUG(this->get_logger(), GetStatusString());
}

void PeriodicMeasurementNode::PublishStatisticMessage()
{
  auto msg = GenerateStatisticMessage(get_name(),
      GetMetricName(),
      window_start_,
      now(),
      GetStatisticsResults());
  publisher_->publish(msg);
}

}  // namespace system_metrics_collector
