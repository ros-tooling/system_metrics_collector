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

#ifndef SYSTEM_METRICS_COLLECTOR__METRIC_DETAILS_INTERFACE_HPP_
#define SYSTEM_METRICS_COLLECTOR__METRIC_DETAILS_INTERFACE_HPP_

#include <string>

namespace system_metrics_collector
{
/**
 * Class to represent a single metric for collection or publication.
 */

class MetricDetailsInterface
{
public:
  virtual ~MetricDetailsInterface() = default;

  virtual std::string GetMetricName() const = 0;

  virtual std::string GetMetricUnit() const = 0;
};

}  // namespace system_metrics_collector

#endif  // SYSTEM_METRICS_COLLECTOR__METRIC_DETAILS_INTERFACE_HPP_
