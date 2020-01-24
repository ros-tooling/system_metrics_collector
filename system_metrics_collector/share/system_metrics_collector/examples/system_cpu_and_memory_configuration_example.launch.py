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
Example launch file demonstrating node configuration.

This launch file uses the system cpu and system memory entry points to start the corresponding
nodes and demonstrates how to configure each node's parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Launch argument names
CPU_NODE_NAME = 'linux_cpu_collector_node_name'
MEMORY_NODE_NAME = 'linux_memory_collector_node_name'
MEASUREMENT_PERIOD = 'measurement_period'
PUBLISH_PERIOD = 'publish_period'
PUBLISH_TOPIC = 'publish_topic'

# Default argument values
default_cpu_node_name = 'linux_cpu_collector'
default_memory_node_name = 'linux_memory_collector'
default_measurement_period_in_ms = '1000'
default_publish_period_in_ms = '60000'
default_publish_topic = 'system_metrics'


def generate_launch_description():
    """Launch system CPU and system memory metrics collector nodes."""
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        CPU_NODE_NAME,
        default_value=default_cpu_node_name,
        description='A custom name for the node that collects the total CPU usage'
                    ' on a Linux system'))
    ld.add_action(DeclareLaunchArgument(
        MEMORY_NODE_NAME,
        default_value=default_memory_node_name,
        description='A custom name for the node that collects the total memory usage'
                    ' on a Linux system'))
    ld.add_action(DeclareLaunchArgument(
        MEASUREMENT_PERIOD,
        default_value=default_measurement_period_in_ms,
        description='The period (in ms) between each subsequent metrics measurement made'
                    ' by the collector nodes'))
    ld.add_action(DeclareLaunchArgument(
        PUBLISH_PERIOD,
        default_value=default_publish_period_in_ms,
        description='The period (in ms) between each subsequent metrics message published'
                    ' by the collector nodes'))
    ld.add_action(DeclareLaunchArgument(
        PUBLISH_TOPIC,
        default_value=default_publish_topic,
        description='The name of the topic to which the collector nodes should publish'))

    node_parameters = [
        {MEASUREMENT_PERIOD: LaunchConfiguration(MEASUREMENT_PERIOD)},
        {PUBLISH_PERIOD: LaunchConfiguration(PUBLISH_PERIOD)}]

    """Run system CPU and memory collector nodes using launch."""
    ld.add_action(
        Node(
            package='system_metrics_collector',
            node_executable='linux_cpu_collector',
            node_name=LaunchConfiguration(CPU_NODE_NAME),
            parameters=node_parameters,
            remappings=[('system_metrics', LaunchConfiguration(PUBLISH_TOPIC))],
            output='screen'))
    ld.add_action(
        Node(
            package='system_metrics_collector',
            node_executable='linux_memory_collector',
            node_name=LaunchConfiguration(MEMORY_NODE_NAME),
            parameters=node_parameters,
            remappings=[('system_metrics', LaunchConfiguration(PUBLISH_TOPIC))],
            output='screen'))

    return ld
