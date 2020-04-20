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
End to end tests for the topic statistics collection.

These tests launch the dummy_talker and topic_statistics_node nodes and use
ROS2 commands to inspect if the expected nodes are active and publishing data.

To run these tests ensure that ROS2 is installed, with the
system_metrics_collector package the relevant setup.bash has been sourced.
"""

import logging
import signal
import subprocess
import sys

import rclpy

import test_helpers


# Expected outputs
EXPECTED_LIFECYCLE_NODES = frozenset(['/topic_stats_collector'])
EXPECTED_REGULAR_NODES = frozenset(['/dummy_talker'])
EXPECTED_LIFECYCLE_STATE = 'active [3]'
EXPECTED_TOPIC = '/system_metrics'
# Commands to execute
LAUNCH_TALKER = 'ros2 launch system_metrics_collector dummy_talker.launch.py'
LAUNCH_COLLECTOR = 'ros2 launch system_metrics_collector topic_statistics_node.launch.py'
# Test constants
TIMEOUT_SECONDS = 30
RETURN_VALUE_FAILURE = 1
RETURN_VALUE_SUCCESS = 0
EXPECTED_NUMBER_OF_MESSAGES_TO_RECEIVE = 5


def main(args=None) -> int:
    """
    Run the topic statistics collector e2e tests. This exits on the first failure encountered.

    :param args:
    :return: 0 if all tests pass, 1 if any fail
    """
    try:
        rclpy.init()

        return_value = RETURN_VALUE_FAILURE

        split_command = LAUNCH_TALKER.split()
        logging.debug('Executing: %s', split_command)
        talker_process = subprocess.Popen(split_command)

        split_command = LAUNCH_COLLECTOR.split()
        logging.debug('Executing: %s', split_command)
        listener_process = subprocess.Popen(split_command)

        logging.info('====Starting tests====')
        test_helpers.check_for_expected_nodes(
            list(EXPECTED_REGULAR_NODES) + list(EXPECTED_LIFECYCLE_NODES))
        test_helpers.check_lifecycle_node_enumeration(EXPECTED_LIFECYCLE_NODES)
        test_helpers.check_lifecycle_node_state(EXPECTED_LIFECYCLE_NODES, EXPECTED_LIFECYCLE_STATE)
        test_helpers.check_for_expected_topic(EXPECTED_TOPIC)
        test_helpers.check_for_statistic_publications(
          EXPECTED_LIFECYCLE_NODES, EXPECTED_NUMBER_OF_MESSAGES_TO_RECEIVE, EXPECTED_TOPIC)
        logging.info('====All tests succeeded====')

        return_value = RETURN_VALUE_SUCCESS

    except test_helpers.SystemMetricsEnd2EndTestException:
        logging.error('Test failure: ', exc_info=True)

    except Exception:
        logging.error('Caught unrelated exception: ', exc_info=True)

    finally:
        logging.debug('Finished tests. Sending SIGINT')
        talker_process.send_signal(signal.SIGINT)
        listener_process.send_signal(signal.SIGINT)
        talker_process.wait(timeout=TIMEOUT_SECONDS)
        listener_process.wait(timeout=TIMEOUT_SECONDS)
        rclpy.shutdown()

    return return_value


if __name__ == '__main__':
    test_helpers.setup_logger()
    test_output = main()
    logging.debug('exiting with test_output=%s', test_output)
    sys.exit(test_output)
