# system_metrics_collector

Note: _This is currently an **experimental** package._

![License](https://img.shields.io/github/license/ros-tooling/system_metrics_collector)
[![GitHub Action Status](https://github.com/ros-tooling/system_metrics_collector/workflows/Test%20system_metrics_collector/badge.svg)](https://github.com/ros-tooling/system_metrics_collector/actions)

## MetricsMessage

This message is used to publish the statistics of measured data points, for example system CPU percentage,
system free memory percentage, message age, etc.

## System Metrics Collector

The goal of this package is to provide lightweight, real-time system metrics to enable
system debugging and diagnosis of ROS2 systems (currently Linux only). It automatically collects
and aggregates CPU % used and memory % used of both system and ROS2 processes.
Data is aggregated in order to provide constant time average, min, max, sample count,
and standard deviation values for each collected metric.

Please see the [package README](system_metrics_collector/README.md) for more details.

## License
The source code is released under an Apache 2.0.

Maintainer: ROS Tooling Working Group, ros-tooling@googlegroups.com
