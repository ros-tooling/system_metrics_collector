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
Helper functions for end to end tests for system_metrics_collector ROS2 package.

These functions are used to assert lifecycle node states.
"""

import logging
import subprocess
import sys
from typing import List

import rclpy
from rclpy.task import Future

from retrying import retry
from statistics_listener import StatisticsListener

# Commands to execute
LIST_NODES_COMMAND = 'ros2 node list'
LIST_SERVICES_COMMAND = 'ros2 service list'
LIST_LIFECYCLE_NODES_COMMAND = 'ros2 lifecycle nodes'
GET_LIFECYCLE_STATE_COMMAND = 'ros2 lifecycle get '
TOPIC_LIST_COMMAND = 'ros2 topic list'
# Test constants
TIMEOUT_SECONDS = 30
PUBLICATION_TEST_TIMEOUT_SECONDS = 180
# Test retry constants
DEFAULT_MAX_ATTEMPTS = 10
DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER = 1000
DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS = 60000
# This is longer than PUBLICATION_TEST_TIMEOUT_SECONDS in order to let the spin
# timout complete
DEFAULT_FIXED_WAIT_MILLISECONDS = (PUBLICATION_TEST_TIMEOUT_SECONDS + TIMEOUT_SECONDS) * 1000


class SystemMetricsEnd2EndTestException(Exception):
    """Exception used to denote end to end test failures."""

    pass


def setup_logger() -> None:
    """Format and setup the logger."""
    logger = logging.getLogger()
    # setting to debug for end to end test soak
    logger.setLevel(logging.DEBUG)

    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(logging.DEBUG)
    formatter = logging.Formatter('[%(module)s] [%(levelname)s] [%(asctime)s]: %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)


def execute_command(command_list: List[str], timeout=TIMEOUT_SECONDS) -> List[str]:
    """
    Execute a command and return its output.

    This uses subprocess.check_output to raise a CalledProcessError for any non-zero command
    return.

    :param command_list: input command to execute
    :param timeout: raise subprocess.TimeoutExpired if the input command exceeds the timeout
    :return: List[str] of the command output
    """
    logging.debug('execute_command: %s', command_list)
    return (subprocess.check_output(command_list, timeout=timeout)
            .decode(sys.stdout.encoding).splitlines())


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_for_expected_nodes(expected_nodes: List) -> None:
    """
    Check that all expected nodes can be found.

    Raise a SystemMetricsEnd2EndTestException if the attempts have been exceeded
    :param args:
    """
    logging.debug('starting test check_for_expected_nodes')

    expected_nodes = list(expected_nodes)
    observed_nodes = execute_command(LIST_NODES_COMMAND.split(' '))
    logging.debug(
        'check_for_expected_nodes=%s observed_nodes=%s', str(expected_nodes), str(observed_nodes))

    if set(expected_nodes).issubset(set(observed_nodes)):
        logging.debug('check_for_expected_nodes success')
        return

    raise SystemMetricsEnd2EndTestException('Failed to enumerate expected nodes.'
                                            ' Observed: ' + str(observed_nodes))


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_lifecycle_node_enumeration(expected_nodes: List) -> None:
    """
    Check that all lifecycle nodes exist.

    This requires the ros2lifecycle dependency.
    """
    logging.debug('starting test check_lifecycle_node_enumeration')

    output = execute_command(LIST_LIFECYCLE_NODES_COMMAND.split(' '))

    if output.sort() == list(expected_nodes).sort():
        logging.info('check_lifecycle_node_enumeration success')
    else:
        raise SystemMetricsEnd2EndTestException('check_lifecycle_node_enumeration failed: '
                                                + str(output))


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_lifecycle_node_state(expected_nodes: List, expected_state: str) -> None:
    """
    Check that each lifecycle node is active.

    This requires the ros2lifecycle dependency.
    """
    logging.debug('starting test check_lifecycle_node_state')

    for lifecycle_node in expected_nodes:

        output = execute_command((GET_LIFECYCLE_STATE_COMMAND
                                  + str(lifecycle_node)).split(' '))

        if not output:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state: Unexpected output'
                                                    ' for node: ' + lifecycle_node)

        if output[0] == expected_state:
            logging.debug('%s in expected state', lifecycle_node)

        else:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state:'
                                                    + lifecycle_node +
                                                    ' not in expected state: '
                                                    + str(output))

    logging.info('check_lifecycle_node_state success')


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_for_expected_topic(expected_topic: str) -> None:
    """
    Check if the expected_topic exists.

    :param expected_topic:
    """
    logging.debug('starting test check_for_expected_topic')

    output = execute_command(TOPIC_LIST_COMMAND.split(' '))

    if expected_topic in output:
        logging.info('check_for_expected_topic success')
    else:
        raise SystemMetricsEnd2EndTestException('Unable to find expected topic: ' + str(output))


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS, wait_fixed=DEFAULT_FIXED_WAIT_MILLISECONDS)
def check_for_statistic_publications(
                                    expected_nodes: List,
                                    num_msgs: int,
                                    expected_topic: str) -> None:
    """
    Check that all nodes publish a statistics message.

    This will suceed fast if all expected messages were published, otherwise
    timeout (default TIMEOUT_SECONDS) if any publishers have not been observed.
    :param args:
    """
    logging.debug('starting test check_for_statistic_publications')
    try:
        future = Future()
        node = StatisticsListener(future, list(expected_nodes), num_msgs, expected_topic)
        rclpy.spin_until_future_complete(node,
                                         future,
                                         timeout_sec=PUBLICATION_TEST_TIMEOUT_SECONDS)

        if node.received_all_expected_messages():
            logging.info('check_for_statistic_publications success')
        else:
            raise SystemMetricsEnd2EndTestException('check_for_statistic_publications failed.'
                                                    ' Absent publisher(s): '
                                                    + str(node.lifecycle_nodes_message_counter))
    finally:
        node.destroy_node()
