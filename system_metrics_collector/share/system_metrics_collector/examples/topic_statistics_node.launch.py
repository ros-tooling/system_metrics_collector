# Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example launch file demonstrating topic statistics node configuration.

This launch file uses the topic statistics subscriber to start the collector
node and demonstrates how to configure the node's parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch argument names
COLLECTOR_NODE_NAME = 'topic_statistics_node_name'
MONITORED_TOPIC_NAME = 'collect_topic_names'
PUBLISH_PERIOD_IN_MS = 'publish_period'
PUBLISH_TOPIC_NAME = 'publish_topic_name'

# Default argument values
DEFAULT_COLLECTOR_NODE_NAME = 'topic_stats_collector'
DEFAULT_MONITORED_TOPIC_NAME = ['dummy_topic']
DEFAULT_PUBLISH_PERIOD_IN_MS = '30000'
DEFAULT_PUBLISH_TOPIC = 'system_metrics'


def generate_launch_description():
    """Launch topic statistics collector nodes."""
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        COLLECTOR_NODE_NAME,
        default_value=DEFAULT_COLLECTOR_NODE_NAME,
        description='A name for the node that collects topic statistics'))
    ld.add_action(DeclareLaunchArgument(
        MONITORED_TOPIC_NAME,
        default_value=DEFAULT_MONITORED_TOPIC_NAME,
        description='The topic to subscribe to in order to measure message metrics'))
    ld.add_action(DeclareLaunchArgument(
        PUBLISH_PERIOD_IN_MS,
        default_value=DEFAULT_PUBLISH_PERIOD_IN_MS,
        description='The period (in ms) between each subsequent metrics message published'
                    ' by the collector node'))
    ld.add_action(DeclareLaunchArgument(
        PUBLISH_TOPIC_NAME,
        default_value=DEFAULT_PUBLISH_TOPIC,
        description='The name of the topic to which the collector nodes should publish'))

    node_parameters = [
        {MONITORED_TOPIC_NAME: LaunchConfiguration(MONITORED_TOPIC_NAME)},
        {PUBLISH_TOPIC_NAME: LaunchConfiguration(PUBLISH_TOPIC_NAME)},
        {PUBLISH_PERIOD_IN_MS: LaunchConfiguration(PUBLISH_PERIOD_IN_MS)}]

    """Run topic statistics collector node using launch."""
    ld.add_action(
        Node(
            package='system_metrics_collector',
            node_executable='topic_statistics_node',
            name=LaunchConfiguration(COLLECTOR_NODE_NAME),
            parameters=node_parameters,
            remappings=[('system_metrics', LaunchConfiguration(PUBLISH_TOPIC_NAME))],
            output='screen')
        )

    return ld
