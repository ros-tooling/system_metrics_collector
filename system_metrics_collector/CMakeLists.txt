cmake_minimum_required(VERSION 3.5)
project(system_metrics_collector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
set(FLAGS -pedantic -Wno-long-long -Wall -Wextra -Wcast-qual -Wformat -Wwrite-strings -Wcast-align -Wno-error=cast-align)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(metrics_statistics_msgs REQUIRED)

set(SOURCES
        src/moving_average_statistics/moving_average.cpp
        src/moving_average_statistics/moving_average.hpp
        src/moving_average_statistics/types.hpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_moving_average_statistics
          test/moving_average_statistics/test_moving_average_statistics.cpp ${SOURCES})

  install(TARGETS test_moving_average_statistics DESTINATION lib/${PROJECT_NAME})
endif()

ament_package()
