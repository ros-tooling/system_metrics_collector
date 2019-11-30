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

#include "collector.hpp"

#include <mutex>
#include <sstream>
#include <string>

#include "../moving_average_statistics/moving_average.hpp"
#include "../moving_average_statistics/types.hpp"

namespace system_metrics_collector
{

bool Collector::start()
{
  std::unique_lock<std::mutex> ulock(mutex);
  if (started_) {
    return false;
  }
  started_ = true;
  return setupStart();
}

bool Collector::stop()
{
  bool ret = false;
  {
    std::unique_lock<std::mutex> ulock(mutex);
    if (!started_) {
      return false;
    }
    started_ = false;

    ret = setupStop();
  }
  clearCurrentMeasurements();
  return ret;
}

void Collector::acceptData(const double measurement)
{
  collected_data_.addMeasurement(measurement);
}

moving_average_statistics::StatisticData Collector::getStatisticsResults() const
{
  return collected_data_.getStatistics();
}

void Collector::clearCurrentMeasurements()
{
  collected_data_.reset();
}

bool Collector::isStarted() const
{
  std::unique_lock<std::mutex> ulock(mutex);
  return started_;
}

std::string Collector::getStatusString() const
{
  std::stringstream ss;
  ss << "started=" << (isStarted() ? "true" : "false") <<
    ", " << statisticsDataToString(getStatisticsResults());
  return ss.str();
}

}  // namespace system_metrics_collector
