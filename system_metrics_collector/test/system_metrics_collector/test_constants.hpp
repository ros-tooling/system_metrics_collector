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

#ifndef SYSTEM_METRICS_COLLECTOR__TEST_CONSTANTS_HPP_
#define SYSTEM_METRICS_COLLECTOR__TEST_CONSTANTS_HPP_

#include <array>
#include <chrono>

#include "system_metrics_collector/proc_pid_cpu_data.hpp"


/**
 * Constants used and shared among the various system metrics collector tests.
 */
namespace test_constants
{
constexpr const std::chrono::milliseconds kMeasurePeriod{50};
constexpr const std::chrono::milliseconds kTestDuration{250};
constexpr const std::chrono::seconds kSpinTimeout{1};
constexpr const std::chrono::seconds kPublishTestTimeout{2};
constexpr const std::chrono::milliseconds kPublishPeriod{150};
constexpr const std::chrono::milliseconds kMeasureCpuPeriod{50};
constexpr const std::chrono::milliseconds kPublishCpuPeriod{6 * 50};

constexpr const char kProcSampleResolutionTest[] =
  "cpu  57211920 335926 18096939 2526329830 14818556 0 1072048 0 0 0\n";
constexpr const std::array<const char *, 6> kProcSamples = {
  "cpu 22451232 118653 7348045 934943300 5378119 0 419114 0 0 0\n",
  "cpu 22451360 118653 7348080 934949227 5378120 0 419117 0 0 0\n",
  "cpu 24343452 61856 6484430 10645595 58695 0 683052 0 0 0\n",
  "cpu 6051294 43322 1611333 9021635 47400 0 177494 0 0 0\n",
  "cpu 6092443 6217 1623536 535731 4143 0 232286 0 0 0\n",
  "cpu 6097071 6498 1612044 544445 3484 0 135942 0 0 0\n",
};
constexpr const double kCpuActiveProcSample_0_1 = 2.7239908106334099;

constexpr const std::array<system_metrics_collector::ProcPidCpuData, 6> kProcPidSamples = {
  system_metrics_collector::ProcPidCpuData{7348045, 22451232},
  system_metrics_collector::ProcPidCpuData{7348080, 22451360},
  system_metrics_collector::ProcPidCpuData{7348100, 22451471},
  system_metrics_collector::ProcPidCpuData{7348112, 22451591},
  system_metrics_collector::ProcPidCpuData{7348240, 22452023},
  system_metrics_collector::ProcPidCpuData{7348245, 22452730},
};
constexpr const double kCpuActiveProcPidSample_0_1 = 27.34375;

constexpr const char kEmptySample[] = "";
constexpr const char kGarbageSample[] = "this is garbage\n";
constexpr const char kIncompleteSample[] =
  "MemTotal:       16302048 kB\n"
  "MemFree:          443300 kB\n";
constexpr const char kIncompleteSample2[] =
  "MemTotal:\n";
constexpr const char kIncompleteSample3[] =
  "MemTotal:       16302048 kB\n"
  "MemFree:          239124 kB\n"
  "MemAvailable:\n";
constexpr const char kIncompleteSample4[] =
  "MemTotal:\n";
constexpr const char kCompleteSample[] =
  "MemTotal:       16302048 kB\n"
  "MemFree:          239124 kB\n"
  "MemAvailable:    9104952 kB\n";
constexpr const char kFullSample[] =
  "MemTotal:       16302048 kB\n"
  "MemFree:          239124 kB\n"
  "MemAvailable:    9104952 kB\n"
  "Buffers:         2755028 kB\n"
  "Cached:          5351344 kB\n"
  "SwapCached:       202440 kB\n"
  "Active:          9743384 kB\n"
  "Inactive:        3662540 kB\n"
  "Active(anon):    5246708 kB\n"
  "Inactive(anon):  1084404 kB\n"
  "Active(file):    4496676 kB\n"
  "Inactive(file):  2578136 kB\n"
  "Unevictable:          68 kB\n"
  "Mlocked:              68 kB\n"
  "SwapTotal:       8003580 kB\n"
  "SwapFree:        6510332 kB\n"
  "Dirty:               436 kB\n"
  "Writeback:             0 kB\n"
  "AnonPages:       5294808 kB\n"
  "Mapped:           823420 kB\n"
  "Shmem:           1037804 kB\n"
  "Slab:            2371932 kB\n"
  "SReclaimable:    2118248 kB\n"
  "SUnreclaim:       253684 kB\n"
  "KernelStack:       21968 kB\n"
  "PageTables:       114360 kB\n"
  "NFS_Unstable:          0 kB\n"
  "Bounce:                0 kB\n"
  "WritebackTmp:          0 kB\n"
  "CommitLimit:    16154604 kB\n"
  "Committed_AS:   19520052 kB\n"
  "VmallocTotal:   34359738367 kB\n"
  "VmallocUsed:           0 kB\n"
  "VmallocChunk:          0 kB\n"
  "HardwareCorrupted:     0 kB\n"
  "AnonHugePages:         0 kB\n"
  "ShmemHugePages:        0 kB\n"
  "ShmemPmdMapped:        0 kB\n"
  "CmaTotal:              0 kB\n"
  "CmaFree:               0 kB\n"
  "HugePages_Total:       0\n"
  "HugePages_Free:        0\n"
  "HugePages_Rsvd:        0\n"
  "HugePages_Surp:        0\n"
  "Hugepagesize:       2048 kB\n"
  "DirectMap4k:     3993192 kB\n"
  "DirectMap2M:    12660736 kB\n"
  "DirectMap1G:     1048576 kB";
constexpr const double kMemoryUsedPercentage = 44.148416198995363;
}  // namespace test_constants

#endif  // SYSTEM_METRICS_COLLECTOR__TEST_CONSTANTS_HPP_
