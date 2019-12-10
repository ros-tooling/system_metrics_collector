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

namespace system_metrics_collector
{
/**
 * Read the contents of the input filename into a string. Helper function
 * for parsing.
 *
 * @param file_name the file to be read
 * @return the file to be read's contents as a std::string
 */
std::string readFileToString(const std::string & file_name);

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
double processMemInfoLines(const std::string & lines);

/**
 * Return the pid of the current process
 * @return the pid of the current process as an int
 */
int getPid();

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__UTILITIES_HPP_
