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
from pathlib import Path
from typing import Set
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import pytest
import rclpy
import retrying

EXPECTED_LIFECYCLE_NODES = [
    '/linux_system_cpu_collector',
    '/linux_system_memory_collector',
    '/listener_process_cpu_node',
    '/listener_process_memory_node',
    '/talker_process_cpu_node',
    '/talker_process_memory_node',
]
EXPECTED_REGULAR_NODES = [
    '/listener',
    '/talker',
]


def include_python_launch_file(package: str, launchfile: str) -> IncludeLaunchDescription:
    package_dir = Path(get_package_share_directory(package))
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_dir / launchfile)))


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        include_python_launch_file(
            'system_metrics_collector', 'examples/talker_listener_example.launch.py'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSystemMetricsLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node('test_system_metrics_nodes')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @retrying.retry(
        stop_max_attempt_number=10,
        wait_exponential_multiplier=1000,
        wait_exponential_max=60000)
    def _test_nodes_exist(self, expected_nodes: Set[str]):
        node_names_and_namespaces = self.node.get_node_names_and_namespaces()
        full_names = {
            namespace + ('' if namespace.endswith('/') else '/') + name
            for name, namespace in node_names_and_namespaces
        }
        self.assertTrue(expected_nodes.issubset(full_names))

    def test_nodes_exist(self):
        return self._test_nodes_exist(set(EXPECTED_LIFECYCLE_NODES + EXPECTED_REGULAR_NODES))
