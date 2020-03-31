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

#ifndef TOPIC_STATISTICS_COLLECTOR__PARAMETER_UTILS_HPP_
#define TOPIC_STATISTICS_COLLECTOR__PARAMETER_UTILS_HPP_

#include <limits>
#include <string>

#include "rcpputils/asserts.hpp"

namespace topic_statistics_collector
{
/**
 * Build a positive integer range to use in parameter descriptors.
 *
 * @return a positive integer range with specififies start, end and step values
 */
static rcl_interfaces::msg::IntegerRange BuildIntegerRange(
  int64_t from,
  int64_t to,
  uint64_t step)
{
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = step;

  return range;
}

/**
 * Build a parameter description for period node parameters.
 *
 * @return a read-only integer range topic descriptor
 */
static rcl_interfaces::msg::ParameterDescriptor BuildPeriodParameterDescriptor(
  const std::string & description)
{
  const auto range = BuildIntegerRange(
    1,
    std::numeric_limits<decltype(rcl_interfaces::msg::IntegerRange::to_value)>::max(),
    1
  );

  rcl_interfaces::msg::ParameterDescriptor period_descriptor;
  period_descriptor.read_only = true;
  period_descriptor.integer_range.push_back(range);
  period_descriptor.description = description;

  return period_descriptor;
}

/**
 * Build a parameter description for string node parameters.
 *
 * @return a read-only topic descriptor
 */
static rcl_interfaces::msg::ParameterDescriptor BuildTopicParameterDescriptor(
  const std::string & description)
{
  rcl_interfaces::msg::ParameterDescriptor topic_descriptor;
  topic_descriptor.description = description;
  topic_descriptor.read_only = true;

  return topic_descriptor;
}

/**
 * Validate the values assigned to string node parameters.
 *
 * @throws std::invalid_argument if the parameter value is empty
 */
static void ValidateStringParam(const std::string & name, const std::string & value)
{
  rcpputils::require_true(!value.empty(), name + " node parameter cannot be empty");
}
}  // namespace topic_statistics_collector
#endif  // TOPIC_STATISTICS_COLLECTOR__PARAMETER_UTILS_HPP_
