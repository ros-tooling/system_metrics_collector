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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "libstatistics_collector/topic_statistics_collector/constants.hpp"
#include "subscriber_topic_statistics.hpp"
#include "system_metrics_collector/msg/dummy_message.hpp"

namespace constants =
  libstatistics_collector::topic_statistics_collector::topic_statistics_constants;
using system_metrics_collector::msg::DummyMessage;
using topic_statistics_collector::SubscriberTopicStatisticsNode;

/**
 * An entry point that starts the topic statistics collector node.
 *
 * This is supposed to be used for demo purposes in topic_statistics_node launch file
 * and in topic statsitics end-to-end tests.
 */

class TopicStatisticsRunner
{
public:
  TopicStatisticsRunner(int argc, char ** argv)
  {
    rclcpp::init(argc, argv);
  }

  ~TopicStatisticsRunner()
  {
    if (topic_stats_node_) {
      topic_stats_node_->deactivate();
    }

    rclcpp::shutdown();
  }

  void run()
  {
    std::vector<std::string> topic_name = {"dummy_data"};
    const auto options = rclcpp::NodeOptions().append_parameter_override(
      constants::kCollectStatsTopicNameParam,
      topic_name /* The topic to which dummy data is published by dummy_talker. */);
    topic_stats_node_ = std::make_shared<SubscriberTopicStatisticsNode<DummyMessage>>(
      "topic_statistics_collector", options);

    rclcpp::executors::MultiThreadedExecutor ex;
    topic_stats_node_->configure();
    topic_stats_node_->activate();

    ex.add_node(topic_stats_node_->get_node_base_interface());
    ex.spin();
  }

private:
  std::shared_ptr<SubscriberTopicStatisticsNode<DummyMessage>> topic_stats_node_;
};

int main(int argc, char ** argv)
{
  TopicStatisticsRunner runner{argc, argv};
  runner.run();

  return 0;
}
