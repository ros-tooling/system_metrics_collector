#!/bin/bash
# This script sources the built ROS2 workspace, installs the necessary Python dependencies,
# and executes the python end to end tests for the system_metrics_collector package.

ROS_WS_DIR=ros_ws/install/
TEST_DIR=ros_ws/src/system_metrics_collector/test

# source the ROS2 installation
source $ROS_WS_DIR/setup.bash && source $ROS_WS_DIR/local_setup.bash
# install the e2e test requirements
pip3 install -r $TEST_DIR/requirements.txt
# run the e2e test
python3 $TEST_DIR/run_e2e_test.py