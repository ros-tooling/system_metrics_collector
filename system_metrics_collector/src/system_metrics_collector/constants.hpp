// Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef SYSTEM_METRICS_COLLECTOR__CONSTANTS_HPP_
#define SYSTEM_METRICS_COLLECTOR__CONSTANTS_HPP_

#include <chrono>
#include <string>

namespace system_metrics_collector
{

namespace collector_node_constants
{

constexpr const char kStatisticsTopicName[] = "system_metrics";

constexpr const char kCollectPeriodParam[] = "measurement_period";
constexpr const std::chrono::milliseconds kDefaultCollectPeriod{1000};    // 1 second

constexpr const char kPublishPeriodParam[] = "publish_period";
constexpr const std::chrono::milliseconds kDefaultPublishPeriod{60000};   // 1 minute

constexpr const char kPercentUnitName[] = "percent";

}  // namespace collector_node_constants

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__CONSTANTS_HPP_
