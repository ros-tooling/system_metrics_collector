#!/bin/bash
# This script sources the built ROS2 workspace and executes the python end to end tests
# for the system_metrics_collector package.
#
# The goal of this script is to independently run end to end tests, as a canary,
# on the master branch and ultimately released APT packages.

ROS_WS_DIR=ros_ws/install
TEST_DIR=ros_ws/src/system_metrics_collector/test

# source the ROS2 installation
source "$ROS_WS_DIR/setup.bash" && source "$ROS_WS_DIR/local_setup.bash"

# run the e2e tests
python3 "$TEST_DIR/system_metrics_e2e_test.py"
python3 "$TEST_DIR/topic_statistics_e2e_test.py"
