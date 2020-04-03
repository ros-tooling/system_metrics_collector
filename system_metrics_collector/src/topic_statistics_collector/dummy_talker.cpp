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
#include <mutex>
#include <utility>

#include "system_metrics_collector/msg/dummy_message.hpp"

#include "rclcpp/rclcpp.hpp"

/**
 * A class that publishes DummyMessages every second with the Header set to current time.
 */
class DummyTalker : public rclcpp::Node
{
public:
  DummyTalker()
  : Node("dummy_talker")
  {
    using namespace std::chrono_literals;
    auto publish_lambda =
      [this]() -> void
      {
        std::unique_lock<std::mutex> ulock{mutex_};

        msg_ = std::make_unique<system_metrics_collector::msg::DummyMessage>();
        msg_->header = std_msgs::msg::Header{};
        msg_->header.stamp = this->now();
        RCLCPP_INFO(
          this->get_logger(),
          "Publishing header: %lu",
          msg_->header.stamp.nanosec);

        publisher_->publish(std::move(msg_));
      };

    publisher_ = this->create_publisher<system_metrics_collector::msg::DummyMessage>(
      "dummy_data",
      10 /* QoS history_depth */);
    timer_ = this->create_wall_timer(1s, publish_lambda);
  }

private:
  std::unique_ptr<system_metrics_collector::msg::DummyMessage> msg_;
  rclcpp::Publisher<system_metrics_collector::msg::DummyMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable std::mutex mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const auto dummy_talker =
    std::make_shared<DummyTalker>();

  rclcpp::executors::SingleThreadedExecutor ex;
  ex.add_node(dummy_talker->get_node_base_interface());
  ex.spin();

  rclcpp::shutdown();

  return 0;
}
