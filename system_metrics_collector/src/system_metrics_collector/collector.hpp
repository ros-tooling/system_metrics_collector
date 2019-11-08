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

#include <mutex>
#include <string>

#include "../moving_average_statistics/moving_average.hpp"
#include "../moving_average_statistics/types.hpp"

#include "rcpputils/thread_safety_annotations.hpp"

/**
 * Simple class in order to collect observed data and generate statistics for the given observations.
 */
class Collector
{
public:
  Collector() = default;
  virtual ~Collector() = default;

  /**
   * Start collecting data. Meant to be called after construction. Note: this locks the recursive mutex class
   * member 'mutex'.
   *
   * @return true if started, false if an error occurred
   */
  bool start();

  /**
   * Stop collecting data. Meant to be a teardown method (before destruction, but should place the
   * class in a restartable state, i.e., start can be called to be able to resume collection.
   *
   * This calls clearCurrentMeasurements.
   *
   * @return true if stopped, false if an error occurred
   */
  bool stop();

  /**
   * Add an observed measurement. This aggregates the measurement and calculates statistics
   * via the moving_average class.
   *
   * @param the measurement observed
   */
  virtual void acceptData(const double measurement);

  /**
   * Return the statistics for all of the observed data.
   *
   * @return the StatisticData for all the observed measurements
   */
  virtual StatisticData getStatisticsResults() const;

  /**
   * Clear / reset all current measurements.
   */
  virtual void clearCurrentMeasurements();

  /**
   * Return true is start has been called, false otherwise.
   *
   * @return the started state of this collector
   */
  bool isStarted() const;

  /**
   * Return a pretty printed status representation of this class
   *
   * @return a string detailing the current status
   */
  virtual std::string getStatusString() const;

  // todo @dabonnie uptime (once start has been called)

private:
  /**
   * Override in order to perform necessary starting steps.
   *
   * @return true if setup was successful, false otherwise.
   */
  virtual bool setupStart() = 0 RCPPUTILS_TSA_REQUIRES(mutex);

  /**
   * Override in order to perform necessary teardown.
   *
   * @return true if teardown was successful, false otherwise.
   */
  virtual bool setupStop() = 0 RCPPUTILS_TSA_REQUIRES(mutex);

  mutable std::mutex mutex;

  MovingAverageStatistics collected_data_;

  bool started_{false} RCPPUTILS_TSA_GUARDED_BY(mutex);
};


#endif  // SYSTEM_METRICS_COLLECTOR__COLLECTOR_HPP_
