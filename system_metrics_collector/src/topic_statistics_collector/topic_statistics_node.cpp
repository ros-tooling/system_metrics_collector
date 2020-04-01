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

#include "rclcpp/rclcpp.hpp"

#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "subscriber_topic_statistics.hpp"
#include "system_metrics_collector/msg/dummy_message.hpp"

/**
 * An entry point that starts the topic statistics collector node.
 */

int main(int argc, char ** argv)
{
  namespace constants =
    libstatistics_collector::topic_statistics_collector::topic_statistics_constants;

  rclcpp::init(argc, argv);

  const auto options = rclcpp::NodeOptions().append_parameter_override(
    constants::kCollectStatsTopicNameParam,
    "dummy_data");
  const auto topic_stats_node =
    std::make_shared<topic_statistics_collector::SubscriberTopicStatisticsNode<
        system_metrics_collector::msg::DummyMessage>>("topic_statistics_collector", options);

  rclcpp::executors::MultiThreadedExecutor ex;
  topic_stats_node->configure();
  topic_stats_node->activate();

  ex.add_node(topic_stats_node->get_node_base_interface());
  ex.spin();

  rclcpp::shutdown();
  topic_stats_node->deactivate();

  return 0;
}
