# system_metrics_collector

Note: _This is currently an **experimental** package._

![License](https://img.shields.io/github/license/ros-tooling/system_metrics_collector)
[![GitHub Action Status](https://github.com/ros-tooling/system_metrics_collector/workflows/Test%20system_metrics_collector/badge.svg)](https://github.com/ros-tooling/system_metrics_collector/actions?query=workflow%3A%22Test+system_metrics_collector%22)
[![End-to-end Testing (Nightly)](https://github.com/ros-tooling/system_metrics_collector/workflows/End-to-end%20Testing%20(Nightly)/badge.svg)](https://github.com/ros-tooling/system_metrics_collector/actions?query=workflow%3A%22End-to-end+Testing+%28Nightly%29%22)

## MetricsMessage

This message is used to publish the statistics of measured data points, for example system CPU percentage,
system free memory percentage, message age, etc.

## System Metrics Collector

The goal of this package is to provide lightweight, real-time system metrics to enable
system debugging and diagnosis of ROS2 systems (currently Linux only). It automatically collects
and aggregates CPU % used and memory % used of both system and ROS2 processes.
Data is aggregated in order to provide constant time average, min, max, sample count,
and standard deviation values for each collected metric.

Please see the [package README](system_metrics_collector/README.md) for more details and usage examples.

## Building from Source

To build from source you'll need to create a new workspace, clone and checkout the latest release branch of
this repository, install all the dependencies, and compile. If you need the latest development features
you can clone from the `master` branch instead of the latest release branch. While we guarantee the release
branches are stable, __the `master` should be considered to have an unstable build__ due to ongoing development.git diff

- Create a ROS2 workspace and a source directory

```sh
mkdir -p ~/ros2-workspace/src
```

- Clone the package into the source directory
```sh
cd ~/ros2-workspace/src
git clone git@github.com:ros-tooling/system_metrics_collector.git
```

- Install dependencies
```sh
cd ~/ros2-workspace
sudo apt-get update && rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

_Note: If building the master branch instead of a release branch you may need to also checkout and build the master branches of the packages this package depends on._

- Build the packages
```sh
cd ~/ros2-workspace && colcon build
```

- Configure ROS2 library Path
```sh
source ~/ros2-workspace/install/local_setup.bash
```

- Run the unit tests
```sh
colcon test && colcon test-result --all
```

## License
The source code is released under an Apache 2.0.

Maintainer: ROS Tooling Working Group, ros-tooling@googlegroups.com
