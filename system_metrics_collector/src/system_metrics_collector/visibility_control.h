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

#ifndef SYSTEM_METRICS_COLLECTOR__VISIBILITY_CONTROL_H_
#define SYSTEM_METRICS_COLLECTOR__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SYSTEM_METRICS_COLLECTOR_EXPORT __attribute__ ((dllexport))
    #define SYSTEM_METRICS_COLLECTOR_IMPORT __attribute__ ((dllimport))
  #else
    #define SYSTEM_METRICS_COLLECTOR_EXPORT __declspec(dllexport)
    #define SYSTEM_METRICS_COLLECTOR_IMPORT __declspec(dllimport)
  #endif
  #ifdef SYSTEM_METRICS_COLLECTOR_BUILDING_DLL
    #define SYSTEM_METRICS_COLLECTOR_PUBLIC SYSTEM_METRICS_COLLECTOR_EXPORT
  #else
    #define SYSTEM_METRICS_COLLECTOR_PUBLIC SYSTEM_METRICS_COLLECTOR_IMPORT
  #endif
  #define SYSTEM_METRICS_COLLECTOR_PUBLIC_TYPE SYSTEM_METRICS_COLLECTOR_PUBLIC
  #define SYSTEM_METRICS_COLLECTOR_LOCAL
#else
  #define SYSTEM_METRICS_COLLECTOR_EXPORT __attribute__ ((visibility("default")))
  #define SYSTEM_METRICS_COLLECTOR_IMPORT
  #if __GNUC__ >= 4
    #define SYSTEM_METRICS_COLLECTOR_PUBLIC __attribute__ ((visibility("default")))
    #define SYSTEM_METRICS_COLLECTOR_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SYSTEM_METRICS_COLLECTOR_PUBLIC
    #define SYSTEM_METRICS_COLLECTOR_LOCAL
  #endif
  #define SYSTEM_METRICS_COLLECTOR_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // SYSTEM_METRICS_COLLECTOR__VISIBILITY_CONTROL_H_
