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

#ifndef MOVING_AVERAGE_STATISTICS__MOVING_AVERAGE_HPP_
#define MOVING_AVERAGE_STATISTICS__MOVING_AVERAGE_HPP_


#include <cmath>

#include <algorithm>
#include <limits>
#include <mutex>
#include <type_traits>

#include "types.hpp"


/**
 *  A class for calculating moving average statistics. This operates in constant memory and constant time. Note:
 *  reset() must be called manually in order to start a new measurement window.
 *
 *  The statistics calculated are average, maximum, minimum, and standard deviation (population).
 *  All are calculated online without storing the observation data. Specifically, the average is a running sum
 *  and the variance is obtained by Welford's online algorithm
 *  (reference: https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford%27s_online_algorithm)
 *  for standard deviation.
 *
 *  When statistics are not available, e.g. no observations have been made, NaNs are returned.
**/

class MovingAverageStatistics
{
public:
  MovingAverageStatistics() = default;
  ~MovingAverageStatistics() = default;

  /**
   *  Returns the arithmetic mean of all data recorded. If no observations have been made, returns NaN.
   *
   *  @return The arithmetic mean of all data recorded, or NaN if the sample count is 0.
  **/
  double average() const
  {
    return getStatistics().average;
  }
  /**
   *  Returns the maximum value recorded. If size of list is zero, returns NaN.
   *
   *  @return The maximum value recorded, or NaN if size of data is zero.
  **/
  double max() const
  {
    return getStatistics().max;
  }
  /**
   *  Returns the minimum value recorded. If size of list is zero, returns NaN.
   *
   *  @return The minimum value recorded, or NaN if size of data is zero.
  **/
  double min() const
  {
    return getStatistics().min;
  }
  /**
   *  Returns the standard deviation (population) of all data recorded. If size of list is zero, returns NaN.
   *
   *  Variance is obtained by Welford's online algorithm,
   *  see https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford%27s_online_algorithm
   *
   *  @return The standard deviation (population) of all data recorded, or NaN if size of data is zero.
  **/
  double standardDeviation() const
  {
    return getStatistics().standard_deviation;
  }
  /**
   *  Return a StatisticData object, containing average, minimum, maximum, standard deviation (population),
   *  and sample count.
   *  For the case of no observations, the average, min, max, and standard deviation are NaN.
   *
   *  @return StatisticData object, containing average, minimum, maximum, standard deviation (population),
   *  and sample count.
  **/
  StatisticData getStatistics() const
  {
    std::lock_guard<std::mutex> guard(mutex);
    StatisticData to_return;

    if (count_ == 0) {
      return to_return;  // already initialized
    }

    // update based on current observations
    to_return.sample_count = count_;
    to_return.average = average_;
    to_return.min = min_;
    to_return.max = max_;
    to_return.standard_deviation = std::sqrt(sum_of_square_diff_from_mean_ / count_);

    return to_return;
  }
  /**
   *  Reset all calculated values. Equivalent to a new window for a moving average.
  **/
  void reset()
  {
    std::lock_guard<std::mutex> guard(mutex);
    average_ = 0;
    min_ = DBL_MAX;
    max_ = DBL_MIN;
    sum_of_square_diff_from_mean_ = 0;
    count_ = 0;
  }
  /**
   *  Observe a sample for the given window. The input item is used to calculate statistics.
   *
   *  @param item The item that was observed
  **/
  virtual void add_measurement(const double & item)
  {
    std::lock_guard<std::mutex> guard(mutex);
    count_++;
    const double previous_average_ = average_;
    average_ = previous_average_ + (item - previous_average_) / count_;
    min_ = std::min(min_, item);
    max_ = std::max(max_, item);
    sum_of_square_diff_from_mean_ = sum_of_square_diff_from_mean_ + (item - previous_average_) *
      (item - average_);
  }

  /**
   * Return the number of samples observed
   *
   * @return the number of samples observed
   */
  int64_t get_count() const
  {
    std::lock_guard<std::mutex> guard(mutex);
    return count_;
  }

private:
  mutable std::mutex mutex;
  // cached values below
  double average_ = 0;
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::min();
  double sum_of_square_diff_from_mean_ = 0;
  int64_t count_ = 0;
};

#endif  // MOVING_AVERAGE_STATISTICS__MOVING_AVERAGE_HPP_
