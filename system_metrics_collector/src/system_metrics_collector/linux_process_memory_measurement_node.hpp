// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP
#define SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP

// read /proc/[pid]/status _> VmSize, or statm (first field): http://man7.org/linux/man-pages/man5/proc.5.html

#include <chrono>
#include <fstream>
#include <sstream>
#include <streambuf>
#include <string>

#include "../../src/system_metrics_collector/linux_memory_measurement_node.hpp"
#include "../../src/system_metrics_collector/periodic_measurement_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"


class LinuxProcessMemoryMeasurementNode : public LinuxProcessMemoryMeasurementNode
{

};
#endif //SYSTEM_METRICS_COLLECTOR_LINUX_PROCESS_MEMORY_MEASUREMENT_NODE_HPP
