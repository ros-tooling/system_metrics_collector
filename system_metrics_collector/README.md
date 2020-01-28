# System metrics collector

## Supported targets
The system metrics collector package supports the following:

* ROS Distro
  * ROS 2: `eloquent`
* OS: `Ubuntu Bionic`

## Description
This package aims to integrate lightweight aggregation tools in order to collect, measure, and publish aggregate metrics.

The [moving average](src/moving_average_statistics/moving_average.hpp)
tools provide constant time sample aggregation to produce average, min, max, standard deviation, and sample count.
The [collector](src/system_metrics_collector/collector.hpp)
provides a thread-safe mechanism to perform measurements.
The [PeriodicMeasurementNode](src/system_metrics_collector/periodic_measurement_node.hpp)
is a [ROS2 Lifecycle](http://design.ros2.org/articles/node_lifecycle.html) [Node](https://github.com/ros2/demos/tree/master/lifecycle)
which utilizes the aggregation tools and provides an abstraction for ROS2: specifically a configurable timer (default 1 second)
is used to perform measurements, which is published (default 1 minute - but configurable) to a topic (default /system_metrics).

The lifecycle state transitions are automatically invoked such that the Node is in an activated state after creation.
The goal is to start metric collection automatically after launching. However, the lifecycle actions (activate, deactivate, shutdown)
can be manually invoked. Please see this [example](#inspect-and-change-lifecycle-state).

## Parameters
There are two parameters defined:

  - **measurement_period**:
```yaml
Parameter name: measurement_period
Type: integer
  Description: The period in milliseconds between each measurement
  Constraints:
    Read only: true
    Min value: 1
    Max value: 9223372036854775807
    Step: 1
```
  - **publish_period**:
```yaml
Parameter name: publish_period
  Type: integer
  Description: The period in milliseconds between each published MetricsMessage. This must be less than
  the measurement_period.
  Constraints:
    Read only: true
    Min value: 1
    Max value: 9223372036854775807
    Step: 1
```

## Usage:

We provide multiple example entry points to use this package in the
[examples directory](/system_metrics_collector/share/system_metrics_collector/examples), which demonstrate how
to run the metric collection nodes and instrument existing nodes in order to measure their performance.

### ROS2 Launch
The [talker_listener_example] launch file demonstrates how to measure the CPU and memory of a ROS2 process.
Specifically, this example instruments the [demo_nodes_cpp] talker and listener nodes and launches them, in separate
processes, with the system CPU and memory measurement nodes. This example can be run using [ros2launch].
```sh
ros2 launch system_metrics_collector talker_listener_example.launch.py
```

The [system_cpu_and_memory_configuration_example] can be launched with [ros2launch], which demonstrates
how to configure the metric collection nodes.
```sh
ros2 launch system_metrics_collector system_cpu_and_memory.launch.py
```

### Manual Execution
You can run the example_main executable which will manually start the system CPU, system memory, process
CPU and process memory measurement nodes. This also sets all the nodes' verbosity levels to DEBUG, which
will print collection and status information to the console. Using [ros2run]
```sh
ros2 run system_metrics_collector example_main
```

Change `publish_period` or `measurement_period` using `--ros-args`:
```sh
ros2 run system_metrics_collector example_main --ros-args -p measurement_period:=100 -publish_period 1000
```

This node will generate 4 nodes. Using [ros2node]
```sh
ros2 node list
/linuxCpuCollector
/linuxMemoryCollector
/linuxProcessCpuCollector
/linuxProcessMemoryCollector
```

## Interaction via the ROS2 CLI
This section describes how to interact with this package using the
[ROS2 Command Line Interface (CLI)](https://github.com/ros2/ros2cli). Example output is provided where relevant.


#### Subscribe to `/system_metrics` to visualize all the statistics.
Using [ros2topic]

```sh
ros2 topic echo /system_metrics
measurement_source_name: linuxMemoryCollector
metrics_source: system_memory_percent_used
unit: percent
window_start:
  sec: 1579638873
  nanosec: 125927653
window_stop:
  sec: 1579638933
  nanosec: 125649059
statistics:
- data_type: 1
  data: 51.665252685546875
- data_type: 3
  data: 51.906898498535156
- data_type: 2
  data: 51.5028190612793
- data_type: 5
  data: 60.0
- data_type: 4
  data: 0.09770704805850983
---
measurement_source_name: linuxCpuCollector
metrics_source: system_cpu_percent_used
unit: percent
window_start:
  sec: 1579638873
  nanosec: 125928189
window_stop:
  sec: 1579638933
  nanosec: 125649074
statistics:
- data_type: 1
  data: 11.075708389282227
- data_type: 3
  data: 22.03821563720703
- data_type: 2
  data: 3.9845757484436035
- data_type: 5
  data: 60.0
- data_type: 4
  data: 5.842845916748047
---
```

#### Review all the available parameters
Using [ros2param]
```sh
ros2 param list
/linuxCpuCollector:
  measurement_period
  publish_period
  use_sim_time
/linuxMemoryCollector:
  measurement_period
  publish_period
  use_sim_time
/linuxProcessCpuCollector:
  measurement_period
  publish_period
  use_sim_time
/linuxProcessMemoryCollector:
  measurement_period
  publish_period
  use_sim_time
```


#### Inspect and change lifecycle state
Using [ros2lifecycle]

List lifecycle nodes:
```sh
ros2 lifecycle nodes
/linuxCpuCollector
/linuxMemoryCollector
```

Get the state of a specific node:
```sh
ros2 lifecycle get /linuxCpuCollector
active [3]
```

Activate the node (start measurement collection and data publishing):
```sh
ros2 lifecycle set /linuxCpuCollector activate
Transitioning successful
```

Deactivate the node (stop measurement collection and data publishing):

```sh
ros2 lifecycle set /linuxCpuCollector deactivate
Transitioning successful
```

[talker_listener_example]: share/system_metrics_collector/examples/talker_listener_example.launch.py
[system_cpu_and_memory_configuration_example]: share/system_metrics_collector/examples/system_cpu_and_memory_configuration_example.launch.py
[ros2launch]: https://github.com/ros2/launch
[demo_nodes_cpp]: https://github.com/ros2/demos/tree/master/demo_nodes_cpp
[ros2launch]: https://github.com/ros2/launch
[ros2lifecycle]: https://github.com/ros2/ros2cli/tree/master/ros2lifecycle
[ros2node]: https://github.com/ros2/ros2cli/tree/master/ros2node
[ros2param]: https://github.com/ros2/ros2cli/tree/master/ros2param
[ros2run]: https://github.com/ros2/ros2cli/tree/master/ros2run
[ros2topic]: https://github.com/ros2/ros2cli/tree/master/ros2topic
