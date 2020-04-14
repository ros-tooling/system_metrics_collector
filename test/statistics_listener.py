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
import logging
from threading import Lock
from typing import List

from statistics_msgs.msg import MetricsMessage

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
        self._logger = logging.getLogger()

        # used to stop spinning upon success
        self._future = future
        # key is node name, value is expected number of messages to receive
        self.lifecycle_nodes_message_counter = Counter()
        for node in expected_lifecycle_nodes:
            self.lifecycle_nodes_message_counter[node] = expected_number_of_messages_to_receive
        self._lock = Lock()

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

        if node_name in self.lifecycle_nodes_message_counter:
            self._logger.debug('received message from %s', node_name)
            received_all_msgs_for_node = False
            with self._lock:
                self.lifecycle_nodes_message_counter[node_name] -= 1
                if self.lifecycle_nodes_message_counter[node_name] == 0:
                    received_all_msgs_for_node = True
                    del self.lifecycle_nodes_message_counter[node_name]

            if received_all_msgs_for_node:
                # don't lock on a logging statement
                self._logger.debug('received all messages from %s', node_name)

        if self.received_all_expected_messages():
            self._logger.debug('received all expected messages')
            self._future.set_result(True)
        else:
            self._logger.debug(
                'messages left to receive %s', self.lifecycle_nodes_message_counter)

    def received_all_expected_messages(self) -> bool:
        """
        Check and return if all expected messages have been received.

        :return: true if all expected messages have been received, specifically
        if the node's received count is not greater than 0, false otherwise.
        """
        return (len(self.lifecycle_nodes_message_counter) == 0)
