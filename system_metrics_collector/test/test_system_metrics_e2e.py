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
import sys

from launch import LaunchDescription
import launch_testing
import pytest

# Allow relative import even though we are not in a real module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from base_metrics_test import include_python_launch_file, TestMetricsBase  # noqa: E402, I100


EXPECTED_LIFECYCLE_NODES = {
    '/linux_system_cpu_collector',
    '/linux_system_memory_collector',
    '/listener_process_cpu_node',
    '/listener_process_memory_node',
    '/talker_process_cpu_node',
    '/talker_process_memory_node',
}
EXPECTED_REGULAR_NODES = {
    '/listener',
    '/talker',
}


@pytest.mark.launch_test
def generate_test_description():
    return LaunchDescription([
        include_python_launch_file(
            'system_metrics_collector', 'examples/talker_listener_example.launch.py'),
        launch_testing.actions.ReadyToTest(),
    ])


class TestSystemMetricsLaunch(TestMetricsBase):
    def test_nodes_exist(self):
        return self._test_nodes_exist(EXPECTED_LIFECYCLE_NODES.union(EXPECTED_REGULAR_NODES))

    def test_lifecycle_nodes_exist(self):
        return self._test_lifecycle_nodes_exist(EXPECTED_LIFECYCLE_NODES)

    def test_lifecycle_nodes_active(self):
        return self._test_lifecycle_nodes_active(EXPECTED_LIFECYCLE_NODES)

    def test_topics_exist(self):
        return self._test_topic_exists('/system_metrics')

    def test_statistic_publication(self):
        return self._test_statistic_publication('/system_metrics', EXPECTED_LIFECYCLE_NODES)
