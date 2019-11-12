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

#include <string>
#include <sstream>

#include "cpu_data.hpp"

/*static*/ constexpr const char ProcCpuData::EMPTY_LABEL[];

size_t ProcCpuData::getIdleTime() const
{
  return times[idle] + times[iOWait];
}

size_t ProcCpuData::getActiveTime() const
{
  return times[user] + times[nice] + times[system] + times[irq] +
         times[softIrq] + times[steal];
}


std::string ProcCpuData::toString()
{
  std::stringstream ss;
  ss << "cpu_label=" << cpu_label <<
    ", user=" << times[user] <<
    ", nice=" << times[nice] <<
    ", system=" << times[system] <<
    ", idle=" << times[idle] <<
    ", iOWait=" << times[iOWait] <<
    ", irq=" << times[irq] <<
    ", softIrq=" << times[softIrq] <<
    ", steal=" << times[steal];
  return ss.str();
}

bool ProcCpuData::isMeasurementEmpty() const
{
  return cpu_label == ProcCpuData::EMPTY_LABEL;
}
