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

"""Instrumented nodes example."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch example instrumented nodes."""
    # Collect, aggregate, and measure system CPU % used
    system_cpu_node = Node(
        package='system_metrics_collector',
        node_executable='linux_cpu_collector',
        output='screen')

    # Collect, aggregate, and measure system memory % used
    system_memory_node = Node(
        package='system_metrics_collector',
        node_executable='linux_memory_collector',
        output='screen'
    )

    # Instrument the listener demo to collect, aggregate, and publish it's CPU % + memory % used
    listener_container = ComposableNodeContainer(
        name='listener_container',
        node_name='listener_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                node_plugin='demo_nodes_cpp::Listener',
                node_name='listener'),
            ComposableNode(
                package='system_metrics_collector',
                node_plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                node_name='listener_process_cpu_node'),
            ComposableNode(
                package='system_metrics_collector',
                node_plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                node_name='listener_process_memory_node')
        ],
        output='screen',
    )

    # Instrument the talker demo to collect, aggregate, and publish it's CPU % + memory % used
    talker_container = ComposableNodeContainer(
        name='talker_container',
        node_name='talker_container',
        node_namespace='',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_nodes_cpp',
                node_plugin='demo_nodes_cpp::Talker',
                node_name='talker'),
            ComposableNode(
                package='system_metrics_collector',
                node_plugin='system_metrics_collector::LinuxProcessCpuMeasurementNode',
                node_name='talker_process_cpu_node'),
            ComposableNode(
                package='system_metrics_collector',
                node_plugin='system_metrics_collector::LinuxProcessMemoryMeasurementNode',
                node_name='talker_process_memory_node')
        ],
        output='screen',
    )

    return LaunchDescription([system_cpu_node,
                              system_memory_node,
                              listener_container,
                              talker_container])
