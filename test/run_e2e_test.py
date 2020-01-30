"""
End to end tests for the system_metrics_collector ROS2 package.

These tests assume that the relevant ROS2 install and project has been sourced.
"""

import logging
import os
import signal
import subprocess
import sys
import time
from typing import List

from metrics_statistics_msgs.msg import MetricsMessage

import rclpy
from rclpy.node import Node
from rclpy.task import Future


EXPECTED_LIFECYCLE_NODES = ['/linux_system_cpu_collector',
                            '/linux_system_memory_collector',
                            '/listener_process_cpu_node',
                            '/listener_process_memory_node',
                            '/talker_process_cpu_node',
                            '/talker_process_memory_node']
EXPECTED_LIFECYCLE_STATE = 'active [3]'
LAUNCH_COMMAND = 'ros2 launch system_metrics_collector talker_listener_example.launch.py'
LIST_LIFECYCLE_NODES_COMMAND = 'ros2 lifecycle nodes'
TIMEOUT_SECONDS = 20
QOS_DEPTH = 1
RETURN_VALUE_FAILURE = 1
RETURN_VALUE_SUCCESS = 1


class SystemMetricsEnd2EndTestException(Exception):
    """Exception used to denote end to end test failures."""

    pass


class StatisticsListener(Node):
    """Listen for MetricsMessages published on the /system_metrics topic."""

    def __init__(self, future: Future, expected_lifecycle_nodes: List):
        super().__init__('statisticsListener')

        self.sub = self.create_subscription(MetricsMessage,
                                            'system_metrics',
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
            self.expected_lifecycle_nodes.remove(node_name)

        if not self.expected_lifecycle_nodes:
            self.received_all_published_stats = True
            self.future.set_result(self.received_all_published_stats)


def check_for_expected_nodes(args=None) -> None:
    """
    Check that all expected nodes can be found.

    :param args:
    """
    try:
        rclpy.init(args=args)
        node = rclpy.create_node('check_for_expected_nodes_test')

        expected_nodes = ['/listener', '/talker'] + EXPECTED_LIFECYCLE_NODES
        observed_nodes = node.get_node_names_and_namespaces()
        # observed_nodes node does not contain the preceeding '/'
        observed_nodes = ['/' + node[0] for node in observed_nodes]

        if set(expected_nodes).issubset(set(observed_nodes)):
            logging.info('check_for_expected_nodes success')
        else:
            raise SystemMetricsEnd2EndTestException('Failed to enumerate expected nodes.'
                                                    ' Found: ', observed_nodes)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def check_lifecycle_node_enumeration() -> None:
    """
    Check that all lifecycle nodes exist.

    This requires the ros2lifecycle dependency.
    """
    output = subprocess.check_output(LIST_LIFECYCLE_NODES_COMMAND.split(' '),
                                     timeout=TIMEOUT_SECONDS)\
        .decode(sys.stdout.encoding).splitlines()

    if output == EXPECTED_LIFECYCLE_NODES:
        logging.info('check_lifecycle_node_enumeration success')
    else:
        raise SystemMetricsEnd2EndTestException('check_lifecycle_node_enumeration failed: '
                                                + str(output))


def check_lifecycle_node_state() -> None:
    """
    Check that each lifecycle node is active.

    This requires the ros2lifecycle dependency.
    """
    for lifecycle_node in EXPECTED_LIFECYCLE_NODES:
        output = subprocess.check_output(['ros2', 'lifecycle', 'get', str(lifecycle_node)],
                                         timeout=TIMEOUT_SECONDS)\
            .decode(sys.stdout.encoding).splitlines()

        if not output:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state: Unexpected output'
                                                    ' for node: ' + lifecycle_node)

        if output[0] == EXPECTED_LIFECYCLE_STATE:
            logging.debug(lifecycle_node + ' in expected state')
        else:
            raise SystemMetricsEnd2EndTestException('check_lifecycle_node_state:'
                                                    + lifecycle_node +
                                                    ' not in expected state: '
                                                    + str(output))

    logging.info('check_lifecycle_node_state success')


def check_for_statistic_publications(args=None) -> None:
    """
    Check that all nodes publish a statistics message.

    :param args:
    """
    try:
        future = Future()
        rclpy.init(args=args)
        node = StatisticsListener(future, set(EXPECTED_LIFECYCLE_NODES))
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


def setup_logger():
    logger = logging.getLogger()
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
        setup_logger()

        return_value = RETURN_VALUE_FAILURE
        split_command = LAUNCH_COMMAND.split()
        logging.info('Executing: ' + str(split_command))
        process = subprocess.Popen(split_command)
        time.sleep(2)

        logging.info('Starting tests')

        check_for_expected_nodes()
        check_lifecycle_node_enumeration()
        check_lifecycle_node_state()
        check_for_statistic_publications(args)

        logging.info('All tests succeeded')
        return_value = RETURN_VALUE_SUCCESS

    except SystemMetricsEnd2EndTestException:
        logging.error('Test failure: ', exc_info=True)

    except Exception:
        logging.error('Caught unrelated error: ', exc_info=True)

    finally:
        logging.info('Finished tests. Sending SIGINT')
        os.kill(process.pid, signal.SIGINT)
        process.wait(timeout=TIMEOUT_SECONDS)

    return return_value


if __name__ == '__main__':
    test_output = main()
    sys.exit(test_output)
