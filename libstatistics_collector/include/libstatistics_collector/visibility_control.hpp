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

#ifndef LIBSTATISTICS_COLLECTOR__VISIBILITY_CONTROL_HPP_
#define LIBSTATISTICS_COLLECTOR__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_EXPORT __attribute__ ((dllexport))
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_IMPORT __attribute__ ((dllimport))
  #else
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_EXPORT __declspec(dllexport)
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_IMPORT __declspec(dllimport)
  #endif
  #ifdef LIBSTATISTICS_COLLECTOR_COMPRESSION_BUILDING_DLL
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC LIBSTATISTICS_COLLECTOR_COMPRESSION_EXPORT
  #else
    #define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC LIBSTATISTICS_COLLECTOR_COMPRESSION_IMPORT
  #endif
  #define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC_TYPE LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC
  #define LIBSTATISTICS_COLLECTOR_COMPRESSION_LOCAL
#else
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_EXPORT __attribute__ ((visibility("default")))
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_IMPORT
#if __GNUC__ >= 4
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC __attribute__ ((visibility("default")))
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_LOCAL  __attribute__ ((visibility("hidden")))
#else
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_LOCAL
#endif
#define LIBSTATISTICS_COLLECTOR_COMPRESSION_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif
#endif  // LIBSTATISTICS_COLLECTOR__VISIBILITY_CONTROL_HPP_
