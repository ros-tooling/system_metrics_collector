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


expected_lifecycle_nodes = ['/linux_system_cpu_collector',
                            '/linux_system_memory_collector',
                            '/listener_process_cpu_node',
                            '/listener_process_memory_node',
                            '/talker_process_cpu_node',
                            '/talker_process_memory_node']

expected_lifecycle_state = 'active [3]'
launch_command = 'ros2 launch system_metrics_collector talker_listener_example.launch.py'


class StatisticsListener(Node):
    """Listen for MetricsMessages published on the /system_metrics topic."""

    def __init__(self, future: Future, expected_lifecycle_nodes: List):
        super().__init__('statisticsListener')

        self.sub = self.create_subscription(MetricsMessage,
                                            'system_metrics',
                                            self.listener_callback,
                                            1)
        self.future = future
        self.received_all_published_stats = False
        self.expected_lifecycle_nodes = expected_lifecycle_nodes

    def listener_callback(self, msg) -> None:
        """
        Handle published MetricsMessages.

        :param msg: received message
        :return: None
        """
        node_name = '/' + msg.measurement_source_name

        if node_name in self.expected_lifecycle_nodes:
            self.expected_lifecycle_nodes.remove(node_name)

        if not self.expected_lifecycle_nodes:
            self.received_all_published_stats = True
            self.future.set_result(self.received_all_published_stats)


def print_test(message: str) -> None:
    """
    Print a formatted string for easier parsing.

    :param message: string to be printed
    :return:
    """
    print('========================')
    print(message)
    print('========================')


def check_for_expected_nodes(args=None) -> None:
    """
    Check that all expected nodes can be found.

    :param args:
    :return:
    """
    expected_nodes = ['/listener', '/talker'] + expected_lifecycle_nodes

    rclpy.init(args=args)
    node = rclpy.create_node('check_for_expected_nodes_test')
    actual_nodes = node.get_node_names_and_namespaces()
    node.destroy_node()
    rclpy.shutdown()

    expected_count = len(expected_nodes)
    actual_count = 0
    for item in actual_nodes:
        found_node = '/' + item[0]
        if found_node in expected_nodes:
            actual_count += 1
            expected_nodes.remove(found_node)

    if expected_count != actual_count:
        raise Exception('Failed to enumerate expected nodes. Found #' + actual_count + ': ', actual_nodes)
    else:
        print_test('check_for_expected_nodes success')


def check_lifecycle_node_enumeration() -> None:
    """
    Check that all lifecycle nodes exist.

    :return:
    """
    stream = os.popen('ros2 lifecycle nodes')
    output = stream.read().splitlines()
    if output != expected_lifecycle_nodes:
        raise Exception('check_lifecycle_node_enumeration failed: ' + str(output))
    else:
        print_test('check_lifecycle_node_enumeration success')


def check_lifecycle_node_state() -> None:
    """
    Check that each lifecycle node is active.

    :return:
    """
    for lifecycle_node in expected_lifecycle_nodes:
        stream = os.popen('ros2 lifecycle get ' + str(lifecycle_node))
        output = stream.read().rstrip()

        if output != expected_lifecycle_state:
            raise Exception('check_lifecycle_node_state:'
                            + lifecycle_node +
                            ' not in expected state: '
                            + str(output))

    print_test('check_lifecycle_node_state success')


def check_for_statistic_publications(args=None) -> None:
    """
    Check that all nodes publish a statistics message.

    :param args:
    :return:
    """
    future = Future()
    rclpy.init(args=args)
    node = StatisticsListener(future, expected_lifecycle_nodes)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)  # todo magic constant
    node.destroy_node()
    rclpy.shutdown()

    if not node.received_all_published_stats:
        raise Exception('check_for_statistic_publications failed. Absent publisher(s): '
                        + str(node.expected_lifecycle_nodes))
    else:
        print_test('check_for_statistic_publications success')


def main(args=None) -> int:
    """
    Run all the e2e tests. This exits on the first failure encountered.

    :param args:
    :return: 0 if all tests pass, 1 if any fail
    """
    retval = 1
    split_command = launch_command.split()
    print_test('Executing: ' + str(split_command))
    process = subprocess.Popen(split_command)
    time.sleep(2)

    try:
        print_test('Starting tests')
        check_for_expected_nodes()
        check_lifecycle_node_enumeration()
        check_lifecycle_node_state()
        check_for_statistic_publications(args)
        print_test('All tests succeeded')
        retval = 0
    except Exception as e:
        print_test('Caught exception: ' + str(e))
        retval = 1
    finally:
        print_test('Finished tests. Sending SIGINT')
        os.kill(process.pid, signal.SIGINT)
        time.sleep(2)

    return retval


if __name__ == '__main__':
    retval = main()
    sys.exit(retval)
