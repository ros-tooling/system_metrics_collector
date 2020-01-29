import os
from typing import List, Tuple
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.task import Future

from metrics_statistics_msgs.msg import MetricsMessage


expected_lifecycle_nodes = [ "/linux_system_cpu_collector",
                             "/linux_system_memory_collector",
                             "/listener_process_cpu_node",
                             "/listener_process_memory_node",
                             "/talker_process_cpu_node",
                             "/talker_process_memory_node",
                             ]

expected_lifecycle_state = "active [3]"
launch_command = "ros2 launch system_metrics_collector talker_listener_example.launch.py"

def print_test(message: str):
    print("========================")
    print(message)
    print("========================")

class StatisticsListener(Node):

    def __init__(self, future: Future):
        super().__init__('statisticsListener')
        self.sub = self.create_subscription(MetricsMessage, 'system_metrics', self.callback, 1)
        self.future = future
        self.received_all_published_stats = False

    def callback(self, msg):
        print(str(msg.measurement_source_name) + str(msg.metrics_source))
        self.future.set_result(True)


def check_for_expected_nodes(args=None):
    expected_nodes = ["/listener", "/talker"] + expected_lifecycle_nodes

    rclpy.init(args=args)
    node = rclpy.create_node('list_all_topics_example')

    actual_nodes = node.get_node_names_and_namespaces()
    node.destroy_node()
    rclpy.shutdown()

    expected_count = len(expected_nodes)
    actual_count = 0
    for item in actual_nodes:
        found_node = "/" + item[0]
        if found_node in expected_nodes:
            actual_count += 1
            expected_nodes.remove(found_node)

    if expected_count != actual_count:
        raise Exception("Failed to enumerate expected nodes. Found: ", actual_nodes)
    else:
        print_test("check_for_expected_nodes sucess")


# check lifecycle node states (activated)
def check_lifecycle_node_enumeration():
    stream = os.popen("ros2 lifecycle nodes")
    output = stream.read().splitlines()
    if output != expected_lifecycle_nodes:
        raise Exception("check_lifecycle_node_enumeration failed")
    else:
        print_test("check_lifecycle_node_enumeration success")

# check lifecycle node states (activated)
def check_lifecycle_node_state():

    for lifecycle_node in expected_lifecycle_nodes:
        stream = os.popen("ros2 lifecycle get " + str(lifecycle_node))
        output = stream.read().rstrip()
        if output != expected_lifecycle_state:
                raise Exception("check_lifecycle_node_state:" + lifecycle_node + " not in expected state: " + str(output))

    print_test("check_for_statistic_publications success")

def check_for_statistic_publications(args=None):

    future = Future()
    rclpy.init(args=args)
    node = StatisticsListener(future)
    rclpy.spin_until_future_complete(node, future, timeout_sec=20)
    node.destroy_node()
    rclpy.shutdown()

    if not node.received_all_published_stats:
        Exception("check_for_statistic_publications failed")
    else:
        print_test("check_for_statistic_publications success")

def main(args=None):
    split_command = launch_command.split()
    print_test("Executing: " + str(split_command))
    process = subprocess.Popen(split_command)
    time.sleep(1)

    print_test("Starting tests")
    #check_for_expected_nodes()
    check_for_statistic_publications(args)
    check_lifecycle_node_enumeration()
    check_lifecycle_node_state()

    print_test("Finished tests. Sending SIGINT")
    os.kill(process.pid, signal.SIGINT)
    time.sleep(2)

if __name__== "__main__":
    main()
