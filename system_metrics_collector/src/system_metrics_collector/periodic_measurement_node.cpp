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
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace system_metrics_collector
{

using metrics_statistics_msgs::msg::MetricsMessage;

constexpr const std::chrono::milliseconds PeriodicMeasurementNode::INVALID_PUBLISH_WINDOW;

PeriodicMeasurementNode::PeriodicMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & publishing_topic,
  const std::chrono::milliseconds publish_period,
  const bool clear_measurements_on_publish)
: Node(name),
  publishing_topic_(publishing_topic),
  measurement_period_(measurement_period),
  publish_period_(publish_period),
  clear_measurements_on_publish_(clear_measurements_on_publish) {}

bool PeriodicMeasurementNode::setupStart()
{
  if (publisher_ == nullptr) {
    publisher_ = create_publisher<MetricsMessage>(publishing_topic_, 10);
  }

  if (!measurement_timer_) {
    RCLCPP_DEBUG(get_logger(), "setupStart: creating measurement_timer_");

    measurement_timer_ = this->create_wall_timer(
      measurement_period_, [this]() {this->performPeriodicMeasurement();});
    measurement_start_ = this->now();
  } else {
    RCLCPP_WARN(this->get_logger(), "setupStart: measurement_timer_ already exists!");
  }

  if (!publish_timer_ && publish_period_ != PeriodicMeasurementNode::INVALID_PUBLISH_WINDOW) {
    RCLCPP_DEBUG(get_logger(), "setupStart: creating publish_timer_");

    publish_timer_ = this->create_wall_timer(
      publish_period_, [this]() {
        this->publishStatistics();
        if (this->clear_measurements_on_publish_) {
          this->clearCurrentMeasurements();
          this->measurement_start_ = this->now();
        }
      });
  } else if (publish_timer_) {
    RCLCPP_WARN(this->get_logger(), "setupStart: publish_timer_ already exists!");
  }

  return true;
}

bool PeriodicMeasurementNode::setupStop()
{
  if (measurement_timer_) {
    measurement_timer_->cancel();
    measurement_timer_.reset();
  }
  if (publish_timer_) {
    publish_timer_->cancel();
    publish_timer_.reset();
  }
  return true;
}

std::string PeriodicMeasurementNode::getStatusString() const
{
  std::stringstream ss;
  ss << "name=" << get_name() <<
    ", measurement_period=" << std::to_string(measurement_period_.count()) << "ms" <<
    ", publishing_topic=" << publishing_topic_ <<
    ", publish_period=" <<
  (publish_period_ != PeriodicMeasurementNode::INVALID_PUBLISH_WINDOW ?
  std::to_string(publish_period_.count()) + "ms" : "None") <<
    ", clear_measurements_on_publish_=" << clear_measurements_on_publish_ <<
    ", " << Collector::getStatusString();
  return ss.str();
}

void PeriodicMeasurementNode::performPeriodicMeasurement()
{
  const double measurement = periodicMeasurement();
  RCLCPP_DEBUG(this->get_logger(), "performPeriodicMeasurement: %f", measurement);

  acceptData(measurement);
  RCLCPP_DEBUG(this->get_logger(), getStatusString());
}

MetricsMessage PeriodicMeasurementNode::newMetricsMessage()
{
  MetricsMessage msg;
  msg.measurement_source_name = get_name();
  msg.window_start = measurement_start_;
  msg.window_stop = now();
  return msg;
}

}  // namespace system_metrics_collector
