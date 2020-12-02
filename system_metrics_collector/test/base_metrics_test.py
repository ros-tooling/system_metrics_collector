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
from collections import Counter
from pathlib import Path
from threading import Lock
from typing import Iterable
from typing import Set
import unittest

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from lifecycle_msgs.msg import State
import rclpy
from rclpy.task import Future
import retrying
import ros2lifecycle.api
import ros2node.api
from statistics_msgs.msg import MetricsMessage


def include_python_launch_file(package: str, launchfile: str) -> IncludeLaunchDescription:
    package_dir = Path(get_package_share_directory(package))
    launchfile_path = str(package_dir / launchfile)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(launchfile_path))


class TestMetricsBase(unittest.TestCase):

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
        wait_exponential_max=10000)
    def _test_nodes_exist(self, expected_nodes: Set[str]):
        node_names = ros2node.api.get_node_names(node=self.node)
        full_names = {n.full_name for n in node_names}
        self.assertTrue(expected_nodes.issubset(full_names))

    @retrying.retry(
        stop_max_attempt_number=10,
        wait_exponential_multiplier=1000,
        wait_exponential_max=10000)
    def _test_lifecycle_nodes_exist(self, expected_nodes: Set[str]) -> None:
        node_names = ros2lifecycle.api.get_node_names(node=self.node)
        full_names = {n.full_name for n in node_names}
        self.assertTrue(expected_nodes.issubset(full_names))

    def _test_lifecycle_nodes_active(self, expected_lifecycle_nodes: Iterable[str]) -> None:
        states = ros2lifecycle.api.call_get_states(
            node=self.node,
            node_names=expected_lifecycle_nodes)
        self.assertTrue(all(s.id == State.PRIMARY_STATE_ACTIVE for s in states.values()))

    def _test_topic_exists(self, topic_name: str) -> None:
        topics_and_types = self.node.get_topic_names_and_types()
        found = False
        for name, types in topics_and_types:
            if name == topic_name:
                found = True
                assert all(t == 'statistics_msgs/msg/MetricsMessage' for t in types)
        self.assertTrue(found, f'No topic named {topic_name}')

    def _test_statistic_publication(self, topic_name: str, expected_nodes: Iterable[str]):
        future = Future()
        message_counter = Counter()
        lock = Lock()
        # arbitrary choice, just tells if it's working for a little while
        expected_messages_per_node = 3
        # we are receiving stats every 10 seconds, so this should pass in 30s
        timeout_sec = 180

        def message_callback(msg):
            node_name = '/' + msg.measurement_source_name
            with lock:
                message_counter[node_name] += 1
                if all(
                    message_counter[node] >= expected_messages_per_node
                    for node in expected_nodes
                ):
                    print('Successfully received all expected messages')
                    future.set_result(True)

        sub = self.node.create_subscription(
            MetricsMessage, topic_name, message_callback, qos_profile=10)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), f'Timed out, received message count: {message_counter}')
        self.node.destroy_subscription(sub)
