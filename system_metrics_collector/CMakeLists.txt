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
set(FLAGS -pedantic -Wno-long-long -Wall -Wextra -Wcast-qual -Wformat -Wwrite-strings -Wcast-align
  -Wno-error=cast-align -Wmissing-declarations)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(WIN32)
  add_definitions(-DNOMINMAX)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(metrics_statistics_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)

add_library(system_metrics_collector SHARED
  src/moving_average_statistics/moving_average.cpp
  src/moving_average_statistics/types.cpp
  src/moving_average_statistics/types.hpp
  src/system_metrics_collector/metrics_message_publisher.cpp
  src/system_metrics_collector/collector.cpp
  src/system_metrics_collector/linux_cpu_measurement_node.cpp
  src/system_metrics_collector/linux_memory_measurement_node.cpp
  src/system_metrics_collector/linux_process_cpu_measurement_node.cpp
  src/system_metrics_collector/linux_process_memory_measurement_node.cpp
  src/system_metrics_collector/periodic_measurement_node.cpp
  src/system_metrics_collector/proc_cpu_data.cpp
  src/system_metrics_collector/utilities.cpp
)
ament_target_dependencies(system_metrics_collector metrics_statistics_msgs rclcpp rcpputils rcutils)
ament_export_libraries(system_metrics_collector)

add_executable(main src/system_metrics_collector/main.cpp)
target_link_libraries(main system_metrics_collector)
ament_target_dependencies(main)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_collector
    test/system_metrics_collector/test_collector.cpp)
  target_link_libraries(test_collector system_metrics_collector)
  ament_target_dependencies(test_collector rclcpp)

  ament_add_gtest(test_metrics_message_publisher
    test/system_metrics_collector/test_metrics_message_publisher.cpp)
  target_link_libraries(test_metrics_message_publisher system_metrics_collector)
  ament_target_dependencies(test_metrics_message_publisher metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_cpu_measurement_node
          test/system_metrics_collector/test_linux_cpu_measurement.cpp)
  target_link_libraries(test_linux_cpu_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_cpu_measurement_node metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_memory_measurement_node
          test/system_metrics_collector/test_linux_memory_measurement.cpp)
  target_link_libraries(test_linux_memory_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_memory_measurement_node metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_process_cpu_measurement_node
          test/system_metrics_collector/test_linux_process_cpu_measurement_node.cpp)
  target_link_libraries(test_linux_process_cpu_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_process_cpu_measurement_node metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_process_memory_measurement_node
          test/system_metrics_collector/test_linux_process_memory_measurement_node.cpp)
  target_link_libraries(test_linux_process_memory_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_process_memory_measurement_node metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_moving_average_statistics
          test/moving_average_statistics/test_moving_average_statistics.cpp)
  target_link_libraries(test_moving_average_statistics system_metrics_collector)
  ament_target_dependencies(test_moving_average_statistics rclcpp)

  ament_add_gtest(test_periodic_measurement_node
    test/system_metrics_collector/test_periodic_measurement_node.cpp)
  target_link_libraries(test_periodic_measurement_node system_metrics_collector)
  ament_target_dependencies(test_periodic_measurement_node metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_utilities
          test/system_metrics_collector/test_utilities.cpp)
  target_link_libraries(test_utilities system_metrics_collector)
  ament_target_dependencies(test_utilities rclcpp)

  install(TARGETS
    test_moving_average_statistics
    DESTINATION
    lib/${PROJECT_NAME})
  install(TARGETS
    test_collector
    DESTINATION
    lib/${PROJECT_NAME})
  install(TARGETS
    test_periodic_measurement_node
    DESTINATION
    lib/${PROJECT_NAME})
endif()

install(TARGETS
  main
  DESTINATION
  lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME})

ament_package()
