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
Instrumented nodes example.

Launches the system cpu measurement node, the system memory measurement node, and
the following as composable nodes: talker, listener, and process cpu and memory for both talker
and listener. All measurements are published to the default /system_metrics topic.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode
from launch_ros.descriptions import ComposableNode


MEASUREMENT_PERIOD = 'measurement_period'
PUBLISH_PERIOD = 'publish_period'
DEFAULT_MEASUREMENT_PERIOD_IN_MS = '1000'
DEFAULT_PUBLISH_PERIOD_IN_MS = '10000'


def generate_launch_description():
    """Launch example instrumented nodes."""
    launch_description = LaunchDescription()

    launch_description.add_action(DeclareLaunchArgument(
        MEASUREMENT_PERIOD,
        default_value=DEFAULT_MEASUREMENT_PERIOD_IN_MS,
        description='The period (in ms) between each subsequent metrics measurement made'
                    ' by the collector nodes'))
    launch_description.add_action(DeclareLaunchArgument(
        PUBLISH_PERIOD,
        default_value=DEFAULT_PUBLISH_PERIOD_IN_MS,
        description='The period (in ms) between each subsequent metrics message published'
                    ' by the collector nodes'))
    node_parameters = [
        {MEASUREMENT_PERIOD: LaunchConfiguration(MEASUREMENT_PERIOD)},
        {PUBLISH_PERIOD: LaunchConfiguration(PUBLISH_PERIOD)}]

    # Collect, aggregate, and measure system CPU % used
    system_cpu_node = LifecycleNode(
        package='system_metrics_collector',
        name='linux_system_cpu_collector',
        node_executable='linux_cpu_collector',
        output='screen',
        parameters=node_parameters,
    )

    # Collect, aggregate, and measure system memory % used
    system_memory_node = LifecycleNode(
        package='system_metrics_collector',
        name='linux_system_memory_collector',
        node_executable='linux_memory_collector',
        output='screen',
        parameters=node_parameters,
    )

    # Instrument the listener demo to collect, aggregate, and publish it's CPU % + memory % used
    listener_container = ComposableNodeContainer(
        name='listener_container',
        namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Listener',
                name='listener'),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                name='listener_process_cpu_node',
                parameters=node_parameters,
            ),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                name='listener_process_memory_node',
                parameters=node_parameters,
            )
        ],
        output='screen',
    )

    # Instrument the talker demo to collect, aggregate, and publish it's CPU % + memory % used
    talker_container = ComposableNodeContainer(
        name='talker_container',
        namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                plugin='demo_nodes_cpp::Talker',
                name='talker'),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                name='talker_process_cpu_node',
                parameters=node_parameters,
            ),
            ComposableNode(
                package='system_metrics_collector',
                plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                name='talker_process_memory_node',
                parameters=node_parameters,
            )
        ],
        output='screen',
    )

    launch_description.add_action(system_memory_node)
    launch_description.add_action(system_cpu_node)
    launch_description.add_action(listener_container)
    launch_description.add_action(talker_container)

    return launch_description
