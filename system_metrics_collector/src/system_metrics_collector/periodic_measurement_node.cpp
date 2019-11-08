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


PeriodicMeasurementNode::PeriodicMeasurementNode(
  const std::string & name,
  const std::chrono::milliseconds measurement_period,
  const std::string & publishing_topic)
: Node(name),
  measurement_period_(measurement_period),
  publishing_topic_(publishing_topic),
  measurement_timer_(nullptr)
{}

bool PeriodicMeasurementNode::setupStart()
{
  if (!measurement_timer_) {
    RCLCPP_DEBUG(this->get_logger(), "creating timer");

    measurement_timer_ = this->create_wall_timer(
      measurement_period_,
      std::bind(&PeriodicMeasurementNode::periodicMeasurement, this));

  } else {
    RCLCPP_WARN(this->get_logger(), "setupStart: measurement_timer_ already exists!");
  }
  return true;
}

bool PeriodicMeasurementNode::setupStop()
{
  if (measurement_timer_) {
    measurement_timer_->cancel();
    measurement_timer_.reset();
  }
  return true;
}

std::string PeriodicMeasurementNode::getStatusString() const
{
  std::stringstream ss;
  ss << "name=" << get_name() <<
    ", measurement_period=" << std::to_string(measurement_period_.count()) << "ms" <<
    ", publishing_topic=" << publishing_topic_ <<
    ", " << Collector::getStatusString();
  return ss.str();
}
