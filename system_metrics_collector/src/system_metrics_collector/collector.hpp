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

#ifndef SYSTEM_METRICS_COLLECTOR__COLLECTOR_HPP_
#define SYSTEM_METRICS_COLLECTOR__COLLECTOR_HPP_

#include "../../src/moving_average_statistics/moving_average.hpp"
#include "../../src/moving_average_statistics/types.hpp"

/**
 * Simple wrapping class in order to collect observed data and generate statistics for the given observations.
 */
class Collector
{
public:
  Collector() = default;
  virtual ~Collector() = default;

  /**
   * Start collecting data. Meant to be called after construction.
   *
   * @return
   */
  virtual bool start() = 0;

  /**
   * Stop collecting data. Meant to be a teardown method (before destruction, but should place the
   * class in a restartable state, i.e., start can be called to be able to resume collection.
   *
   * @return
   */
  virtual bool stop() = 0;

  /**
   * Add an observed measurement.
   *
   * @param measurement
   */
  virtual void acceptData(double measurement);

  /**
   * Return the statistics for all of the observed data.
   *
   * @return
   */
  virtual StatisticData getStatisticsResults() const;

  /**
   * Clear / reset all current measurements.
   */
  virtual void clearCurrentMeasurements();

private:
  MovingAverageStatistics collected_data_;
  bool started_ = false;
};


#endif  // SYSTEM_METRICS_COLLECTOR__COLLECTOR_HPP_
