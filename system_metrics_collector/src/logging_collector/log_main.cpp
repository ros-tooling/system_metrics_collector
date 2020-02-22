//
// Created by dbbonnie on 2/21/20.
//

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

#include "logging_collector.hpp"

using namespace logging_collector;
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoggingCollector>());
  rclcpp::shutdown();
  return 0;
}