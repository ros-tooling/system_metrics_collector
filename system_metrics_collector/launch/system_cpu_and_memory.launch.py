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

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Argument names
CPU_NODE_NAME = 'linux_cpu_collector_node_name'
MEMORY_NODE_NAME = 'linux_memory_collector_node_name'
MEASUREMENT_PERIOD = 'measurement_period'
PUBLISH_PERIOD = 'publish_period'
PUBLISH_TOPIC = 'publish_topic'

"""Launch system CPU and system memory metrics collector nodes."""
def generate_launch_description():
    # Default to included config file
    config_filepath = os.path.join(
        get_package_share_directory('system_metrics_collector'),
        'config',
        'system_metrics_collector_config.yaml')
    with open(config_filepath, 'r') as f:
        config_file_contents = f.read()
    config_yaml_data = yaml.safe_load(config_file_contents)

    config_parameters = config_yaml_data['system_metrics_collector']['ros__parameters']
    cpu_node_name = str(config_parameters[CPU_NODE_NAME])
    memory_node_name = str(config_parameters[MEMORY_NODE_NAME])
    measurement_period = str(config_parameters[MEASUREMENT_PERIOD])
    publish_period = str(config_parameters[PUBLISH_PERIOD])
    publish_topic = str(config_parameters[PUBLISH_TOPIC])

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(CPU_NODE_NAME, default_value=cpu_node_name))
    ld.add_action(DeclareLaunchArgument(MEMORY_NODE_NAME, default_value=memory_node_name))
    ld.add_action(DeclareLaunchArgument(MEASUREMENT_PERIOD, default_value=measurement_period))
    ld.add_action(DeclareLaunchArgument(PUBLISH_PERIOD, default_value=publish_period))
    ld.add_action(DeclareLaunchArgument(PUBLISH_TOPIC, default_value=publish_topic))

    node_parameters = list()
    node_parameters.append({MEASUREMENT_PERIOD : LaunchConfiguration(MEASUREMENT_PERIOD)})
    node_parameters.append({PUBLISH_PERIOD : LaunchConfiguration(PUBLISH_PERIOD)})

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
