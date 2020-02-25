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

#ifndef SYSTEM_METRICS_COLLECTOR__UTILITIES_HPP_
#define SYSTEM_METRICS_COLLECTOR__UTILITIES_HPP_

#include <string>

#include "proc_cpu_data.hpp"
#include "proc_pid_cpu_data.hpp"

namespace system_metrics_collector
{
/**
 * Read the contents of the input filename into a string. Helper function
 * for parsing.
 *
 * @param file_name the file to be read
 * @return the file to be read's contents as a std::string
 */
std::string ReadFileToString(const std::string & file_name);

/**
 * Parse a line read from /proc/stat
 *
 * @param stat_cpu_line a line from /proc/stat
 * @return ProcCpuData struct populated if parsed, otherwise empty
 */
ProcCpuData ProcessStatCpuLine(const std::string & stat_cpu_line);

/**
 * Measures Linux process-specific and system-wide CPU times using clock_gettime()
 *
 * @return ProcPidCpuData struct populated if measurements were successful, otherwise empty
 */
ProcPidCpuData MeasurePidCpuTime();

/**
 * Compute the percentage of CPU active.
 *
 * @param measurement1 the first measurement
 * @param measurement2 the second measurement (made after the first)
 * @return percentage of CPU active
 */
double ComputeCpuActivePercentage(
  const ProcCpuData & measurement1,
  const ProcCpuData & measurement2);

/**
 * Computes the percentage of CPU active for a given process.
 * The input measurements contain data about the total amount of CPU time that the process and
 * the entire system were active for. The calculation is to find the delta of the process and
 * the system CPU times, and then the process CPU time as a percentage of the system CPU time.
 *
 * @param measurement1 the first measurement for the process and system
 * @param measurement2 the second measurement for the process and system (made after the first)
 * @return percentage of CPU active for the process
 */
double ComputePidCpuActivePercentage(
  const ProcPidCpuData & measurement1,
  const ProcPidCpuData & measurement2);

/**
 * Process input lines from the /proc/meminfo file. The expected format to
 * compute memory used is
 *
 *    MemTotal:       16302048 kB
 *    MemFree:          200660 kB
 *    MemAvailable:    9523668 kB
 *
 * @param lines the lines from the /proc/meminfo file
 * @return the percentage of memory used, specifically
 * (MemTotal - MemAvailable) / MemTotal * 100.0
 */
double ProcessMemInfoLines(const std::string & lines);

/**
 * Return the pid of the current process
 * @return the pid of the current process as an int
 */
int GetPid();

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__UTILITIES_HPP_
