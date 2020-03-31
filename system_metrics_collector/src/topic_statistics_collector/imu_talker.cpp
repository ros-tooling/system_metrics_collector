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

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;

/**
 * A class that publishes IMU messages every second with the Header set to current time.
 */
class ImuTalker : public rclcpp::Node
{
public:
  ImuTalker()
  : Node("imu_talker")
  {
    auto publish_lambda =
      [this]() -> void
      {
        msg_ = std::make_unique<sensor_msgs::msg::Imu>();
        msg_->header = std_msgs::msg::Header{};
        msg_->header.stamp = this->now();
        RCLCPP_INFO(
          this->get_logger(),
          "%lu Publishing header: %lu",
          ++count_,
          msg_->header.stamp.nanosec);

        publisher_->publish(std::move(msg_));
      };

    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
      "imu_data",
      10);
    timer_ = this->create_wall_timer(1s, publish_lambda);
  }

private:
  uint64_t count_ = 1;
  std::unique_ptr<sensor_msgs::msg::Imu> msg_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto imu_talker_node =
    std::make_shared<ImuTalker>();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(imu_talker_node->get_node_base_interface());
  ex.spin();

  rclcpp::shutdown();

  return 0;
}
