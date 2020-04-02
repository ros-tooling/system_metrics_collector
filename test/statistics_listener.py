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

import logging
import sys
from threading import Lock
from typing import List

from metrics_statistics_msgs.msg import MetricsMessage

import rclpy
from rclpy.node import Node
from rclpy.task import Future

# QoS history_depth to use for listener node
QOS_DEPTH = 10

"""
Helper class for running system meterics end-to-end tests.

This class is a Node that subscribes to the topic to which MetricsMessages are published
and checks that all expected messages are received.
"""

class StatisticsListener(Node):
    """Listen for MetricsMessages published on the /system_metrics topic."""

    def __init__(self, future: Future, expected_lifecycle_nodes: List,
                 expected_number_of_messages_to_receive: int, subscription_topic: str):
        super().__init__('statisticsListener')

        self.sub = self.create_subscription(MetricsMessage,
                                            subscription_topic,
                                            self.listener_callback,
                                            QOS_DEPTH)
        # set up logging
        logger = logging.getLogger()
        # setting to debug for end to end test soak
        logger.setLevel(logging.DEBUG)

        handler = logging.StreamHandler(sys.stdout)
        handler.setLevel(logging.DEBUG)
        formatter = logging.Formatter('[%(module)s] [%(levelname)s] [%(asctime)s]: %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        # used to stop spinning upon success
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
                    removed = True
            if removed:
                # don't lock on a logging statement
                logging.debug('received all messages from %s', node_name)

        if self.received_all_expected_messages():
            logging.debug('received all expected messages')
            self.future.set_result(True)
        else:
            logging.debug('messages left to receive %s', self.expected_lifecycle_nodes_dict)

    def received_all_expected_messages(self) -> bool:
        """
        Check and return if all expected messages have been received.

        :return: true if all expected messages have been received, specifically
        if the node's received count is not greater than 0, false otherwise.
        """
        return not any(v > 0 for _, v in self.expected_lifecycle_nodes_dict.items())
