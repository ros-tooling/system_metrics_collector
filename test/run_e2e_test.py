"""
End to end tests for the system_metrics_collector ROS2 package.

These tests launch the talker_listener_example and use ROS2 to inspect if the
expected nodes are active and publishing data. To run these tests ensure that
ROS2 is installed, with the system_metrics_collector package the relevant
setup.bash has been sourced.
"""

import logging
import signal
import subprocess
import sys
from threading import Lock
from typing import List

from metrics_statistics_msgs.msg import MetricsMessage

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from retrying import retry

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
LIST_NODES_COMMAND = 'ros2 node list'
LIST_SERVICES_COMMAND = 'ros2 service list'
LAUNCH_COMMAND = 'ros2 launch system_metrics_collector talker_listener_example.launch.py'
LIST_LIFECYCLE_NODES_COMMAND = 'ros2 lifecycle nodes'
GET_LIFECYCLE_STATE_COMMAND = 'ros2 lifecycle get '
TOPIC_LIST_COMMAND = 'ros2 topic list'
# test constants
TIMEOUT_SECONDS = 30
QOS_DEPTH = 1
RETURN_VALUE_FAILURE = 1
RETURN_VALUE_SUCCESS = 0
EXPECTED_NUMBER_OF_MESSAGES_TO_RECEIVE = 5
PUBLICATION_TEST_TIMEOUT_SECONDS = 180
# retry constants
DEFAULT_MAX_ATTEMPTS = 10
DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER = 1000
DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS = 60000
DEFAULT_FIXED_WAIT_MILLISECONDS = (PUBLICATION_TEST_TIMEOUT_SECONDS + TIMEOUT_SECONDS) * 1000


class SystemMetricsEnd2EndTestException(Exception):
    """Exception used to denote end to end test failures."""

    pass


class StatisticsListener(Node):
    """Listen for MetricsMessages published on the /system_metrics topic."""

    def __init__(self, future: Future, expected_lifecycle_nodes: List,
                 expected_number_of_messages_to_receive: int):
        super().__init__('statisticsListener')

        self.sub = self.create_subscription(MetricsMessage,
                                            EXPECTED_TOPIC,
                                            self.listener_callback,
                                            QOS_DEPTH)
        self.future = future
        # key is node name, value is expected number of messages to receive
        self.expected_lifecycle_nodes_dict = {}
        for node in expected_lifecycle_nodes:
            self.expected_lifecycle_nodes_dict[node] = expected_number_of_messages_to_receive
        self.lock = Lock()

    def listener_callback(self, msg) -> None:
        """
        Handle published MetricsMessages.

        Checks each received message measurement_source_name against a dict of
        expected_lifecycle_nodes, decrements each entry for its message received,
        and marks the future field as done if all sources have been observed.
        :param msg: received message
        :return: None
        """
        node_name = '/' + msg.measurement_source_name

        if node_name in self.expected_lifecycle_nodes_dict:
            logging.debug('received message from %s', node_name)
            removed = False
            with self.lock:
                self.expected_lifecycle_nodes_dict[node_name] = (
                        self.expected_lifecycle_nodes_dict[node_name] - 1)
                if self.expected_lifecycle_nodes_dict[node_name] == 0:
                    del self.expected_lifecycle_nodes_dict[node_name]
                    removed = True
            if removed:
                logging.debug('received all messages from %s', node_name)

        if self.received_all_expected_messages():
            logging.debug('received all expected messages')
            self.future.set_result(True)
        else:
            logging.debug('messages left to receive %s', self.expected_lifecycle_nodes_dict)

    def received_all_expected_messages(self) -> bool:
        """
        Check and return if all expected messages have been received.

        :return: true if all expected messages have been received, false otherwise
        """
        return not self.expected_lifecycle_nodes_dict


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
def check_for_expected_nodes(args=None) -> None:
    """
    Check that all expected nodes can be found.

    Raise a SystemMetricsEnd2EndTestException if the attempts have been exceeded
    :param args:
    """
    logging.debug('starting test check_for_expected_nodes')

    expected_nodes = ['/listener', '/talker'] + list(EXPECTED_LIFECYCLE_NODES)
    observed_nodes = execute_command(LIST_NODES_COMMAND.split(' '))
    logging.debug('check_for_expected_nodes observed_nodes=%s', str(observed_nodes))

    if set(expected_nodes).issubset(set(observed_nodes)):
        logging.debug('check_for_expected_nodes success')
        return

    raise SystemMetricsEnd2EndTestException('Failed to enumerate expected nodes.'
                                            ' Observed: ' + str(observed_nodes))


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_lifecycle_node_enumeration() -> None:
    """
    Check that all lifecycle nodes exist.

    This requires the ros2lifecycle dependency.
    """
    logging.debug('starting test check_lifecycle_node_enumeration')

    output = execute_command(LIST_LIFECYCLE_NODES_COMMAND.split(' '))

    if output.sort() == list(EXPECTED_LIFECYCLE_NODES).sort():
        logging.info('check_lifecycle_node_enumeration success')
    else:
        raise SystemMetricsEnd2EndTestException('check_lifecycle_node_enumeration failed: '
                                                + str(output))


@retry(stop_max_attempt_number=DEFAULT_MAX_ATTEMPTS,
       wait_exponential_multiplier=DEFAULT_WAIT_EXPONENTIAL_MULTIPLIER,
       wait_exponential_max=DEFAULT_MAX_EXPONENTIAL_WAIT_MILLISECONDS)
def check_lifecycle_node_state() -> None:
    """
    Check that each lifecycle node is active.

    This requires the ros2lifecycle dependency.
    """
    logging.debug('starting test check_lifecycle_node_state')

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
def check_for_statistic_publications(args=None) -> None:
    """
    Check that all nodes publish a statistics message.

    This will suceed fast if all expected messages were published, otherwise
    timeout (default TIMEOUT_SECONDS) if any publishers have not been observed.
    :param args:
    """
    logging.debug('starting test check_for_statistic_publications')
    try:
        future = Future()
        rclpy.init(args=args)
        node = StatisticsListener(future,
                                  list(EXPECTED_LIFECYCLE_NODES),
                                  EXPECTED_NUMBER_OF_MESSAGES_TO_RECEIVE)
        rclpy.spin_until_future_complete(node,
                                         future,
                                         timeout_sec=PUBLICATION_TEST_TIMEOUT_SECONDS)

        if node.received_all_expected_messages():
            logging.info('check_for_statistic_publications success')
        else:
            raise SystemMetricsEnd2EndTestException('check_for_statistic_publications failed.'
                                                    ' Absent publisher(s): '
                                                    + str(node.expected_lifecycle_nodes_dict))
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
        process.send_signal(signal.SIGINT)
        process.wait(timeout=TIMEOUT_SECONDS)

    return return_value


if __name__ == '__main__':
    setup_logger()
    test_output = main()
    logging.debug('exiting with test_output=%s', test_output)
    sys.exit(test_output)
