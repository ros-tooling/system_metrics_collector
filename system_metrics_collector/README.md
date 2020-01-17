# System metric collector

## Parameters

There are two parameters defined:

  - **measurement_period**:
```sh
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
```sh
Parameter name: publish_period
  Type: integer
  Description: The period in milliseconds between each published MetricsMessage
  Constraints:
    Read only: true
    Min value: 1
    Max value: 9223372036854775807
    Step: 1
```

## Usage:

You can run the following command to generate CPU and Memory statistics. for both, all the system and the node running:

```
ros2 run system_metrics_collector main
```

Change `publish_period` or `measurement_period` using `--ros-args`:

```
ros2 run system_metrics_collector main --ros-args -p measurement_period:=100 -publish_period 1000
```

This node will generate 4 nodes:

```
ros2 node list
/linuxCpuCollector
/linuxMemoryCollector
/linuxProcessCpuCollector
/linuxProcessMemoryCollector
```

Subscribe to `/system_metrics` to visualize all the statistics:

```
ros2 topic echo /system_metrics
```

Review all the available parameters with the following command:

```
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
