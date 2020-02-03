"""
End to end tests for the system_metrics_collector ROS2 package.

These tests launch the talker_listener_example and use ROS2 to inspect if the
expected nodes are active and publishing data. To run these tests ensure that
ROS2 is installed, with the system_metrics_collector package the relevant
setup.bash has been sourced.
"""

import logging
import os
import signal
import subprocess
import sys
import time
from typing import List

from retry import retry

from metrics_statistics_msgs.msg import MetricsMessage

import rclpy
from rclpy.node import Node
from rclpy.task import Future

# expected outputs
EXPECTED_LIFECYCLE_NODES = frozenset(['/linux_system_cpu_collector',
                                      '/linux_system_memory_collector',
                                      '/listener_process_cpu_node',
                                      '/listener_process_memory_node',
                                      '/talker_process_cpu_node',
                                      '/talker_process_memory_node'])
EXPECTED_LIFECYCLE_STATE = 'active [3]'
EXPECTED_TOPIC = '/system_metrics'
# executed commands
LIST_SERVICES_COMMAND = 'ros2 service list'
LAUNCH_COMMAND = 'ros2 launch system_metrics_collector talker_listener_example.launch.py'
LIST_LIFECYCLE_NODES_COMMAND = 'ros2 lifecycle nodes'
GET_LIFECYCLE_STATE_COMMAND = 'ros2 lifecycle get '
TOPIC_LIST_COMMAND = 'ros2 topic list'
# test constants
TIMEOUT_SECONDS = 20
QOS_DEPTH = 1
RETURN_VALUE_FAILURE = 1
RETURN_VALUE_SUCCESS = 1
# retry constants
DEFAULT_DELAY_SECONDS = 2
DEFAULT_BACKOFF = 2
DEFAULT_TRIES = 5


class SystemMetricsEnd2EndTestException(Exception):
    """Exception used to denote end to end test failures."""

    pass


class StatisticsListener(Node):
    """Listen for MetricsMessages published on the /system_metrics topic."""

    def __init__(self, future: Future, expected_lifecycle_nodes: List):
        super().__init__('statisticsListener')

        self.sub = self.create_subscription(MetricsMessage,
                                            EXPECTED_TOPIC,
                                            self.listener_callback,
                                            QOS_DEPTH)
        self.future = future
        self.received_all_published_stats = False
        self.expected_lifecycle_nodes = expected_lifecycle_nodes

    def listener_callback(self, msg) -> None:
        """
        Handle published MetricsMessages.

        Checks each received message measurement_source_name against a list of
        expected_lifecycle_nodes and marks the future field as done if all sources
        have been observed.
        :param msg: received message
        :return: None
        """
        node_name = '/' + msg.measurement_source_name

        if node_name in self.expected_lifecycle_nodes:
            logging.debug('received message from %s', node_name)
            self.expected_lifecycle_nodes.remove(node_name)

        if not self.expected_lifecycle_nodes:
            logging.debug('received all expected messages')
            self.received_all_published_stats = True
            self.future.set_result(self.received_all_published_stats)


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


@retry(SystemMetricsEnd2EndTestException, tries=DEFAULT_TRIES, delay=DEFAULT_DELAY_SECONDS, backoff=DEFAULT_BACKOFF)
def check_for_expected_nodes(args=None) -> None:
    """
    Check that all expected nodes can be found.

    raise a SystemMetricsEnd2EndTestException if the attempts have been exceeded
    :param args:
    """
    try:
        rclpy.init(args=args)
        node = rclpy.create_node('check_for_expected_nodes_test')
        expected_nodes = ['/listener', '/talker'] + list(EXPECTED_LIFECYCLE_NODES)

        observed_nodes = node.get_node_names_and_namespaces()
        # an observed_node does not contain the preceding '/'
        observed_nodes = ['/' + node[0] for node in observed_nodes]

        if set(expected_nodes).issubset(set(observed_nodes)):
            logging.debug('check_for_expected_nodes success')
            return

        raise SystemMetricsEnd2EndTestException('Failed to enumerate expected nodes.'
                                                ' Observed: ' + str(observed_nodes))
    finally:
        node.destroy_node()
        rclpy.shutdown()


@retry(SystemMetricsEnd2EndTestException, tries=DEFAULT_TRIES, delay=DEFAULT_DELAY_SECONDS, backoff=DEFAULT_BACKOFF)
def check_lifecycle_node_enumeration() -> None:
    """
    Check that all lifecycle nodes exist.

    This requires the ros2lifecycle dependency.
    """
    output = execute_command(LIST_LIFECYCLE_NODES_COMMAND.split(' '))

    if output.sort() == list(EXPECTED_LIFECYCLE_NODES).sort():
        logging.info('check_lifecycle_node_enumeration success')
    else:
        raise SystemMetricsEnd2EndTestException('check_lifecycle_node_enumeration failed: '
                                                + str(output))


@retry(SystemMetricsEnd2EndTestException, tries=DEFAULT_TRIES, delay=DEFAULT_DELAY_SECONDS, backoff=DEFAULT_BACKOFF)
def check_lifecycle_node_state() -> None:
    """
    Check that each lifecycle node is active.

    This requires the ros2lifecycle dependency.
    """
    for lifecycle_node in EXPECTED_LIFECYCLE_NODES:

        output = execute_command((GET_LIFECYCLE_STATE_COMMAND
                                  + str(lifecycle_node)).split(' '))

        if not output:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state: Unexpected output'
                                                    ' for node: ' + lifecycle_node)

        if output[0] == EXPECTED_LIFECYCLE_STATE:
            logging.debug('%s in expected state', lifecycle_node)

        else:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state:'
                                                    + lifecycle_node +
                                                    ' not in expected state: '
                                                    + str(output))

    logging.info('check_lifecycle_node_state success')


@retry(SystemMetricsEnd2EndTestException, tries=DEFAULT_TRIES, delay=DEFAULT_DELAY_SECONDS, backoff=DEFAULT_BACKOFF)
def check_for_expected_topic(expected_topic: str) -> None:
    """
    Check if the expected_topic exists.

    :param expected_topic:
    """
    output = execute_command(TOPIC_LIST_COMMAND.split(' '))

    if expected_topic in output:
        logging.info('check_for_expected_topic success')
    else:
        raise SystemMetricsEnd2EndTestException('Unable to find expected topic: ' + str(output))


@retry(SystemMetricsEnd2EndTestException, tries=DEFAULT_TRIES, delay=DEFAULT_DELAY_SECONDS, backoff=DEFAULT_BACKOFF)
def check_for_statistic_publications(args=None) -> None:
    """
    Check that all nodes publish a statistics message.

    This will timeout (default TIMEOUT_SECONDS) if any publishers have not been observed.
    :param args:
    """
    try:
        future = Future()
        rclpy.init(args=args)
        node = StatisticsListener(future, list(EXPECTED_LIFECYCLE_NODES))
        rclpy.spin_until_future_complete(node, future, timeout_sec=TIMEOUT_SECONDS)

        if node.received_all_published_stats:
            logging.info('check_for_statistic_publications success')
        else:
            raise SystemMetricsEnd2EndTestException('check_for_statistic_publications failed.'
                                                    ' Absent publisher(s): '
                                                    + str(node.expected_lifecycle_nodes))
    finally:
        node.destroy_node()
        rclpy.shutdown()


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


def main(args=None) -> int:
    """
    Run all the e2e tests. This exits on the first failure encountered.

    :param args:
    :return: 0 if all tests pass, 1 if any fail
    """
    try:
        return_value = RETURN_VALUE_FAILURE
        split_command = LAUNCH_COMMAND.split()
        logging.debug('Executing: %s', split_command)
        process = subprocess.Popen(split_command)

        logging.info('====Starting tests====')
        check_for_expected_nodes()
        check_lifecycle_node_enumeration()
        check_lifecycle_node_state()
        check_for_expected_topic(EXPECTED_TOPIC)
        check_for_statistic_publications(args)
        logging.info('====All tests succeeded====')

        return_value = RETURN_VALUE_SUCCESS

    except SystemMetricsEnd2EndTestException:
        logging.error('Test failure: ', exc_info=True)

    except Exception:
        logging.error('Caught unrelated exception: ', exc_info=True)

    finally:
        logging.debug('Finished tests. Sending SIGINT')
        os.kill(process.pid, signal.SIGINT)
        process.wait(timeout=TIMEOUT_SECONDS)

    return return_value


if __name__ == '__main__':
    setup_logger()
    test_output = main()
    sys.exit(test_output)
