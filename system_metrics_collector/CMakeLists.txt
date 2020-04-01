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
find_package(libstatistics_collector REQUIRED)
find_package(message_filters REQUIRED)
find_package(metrics_statistics_msgs REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rcutils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(src)

add_library(system_metrics_collector SHARED
  src/system_metrics_collector/metrics_message_publisher.cpp
  src/system_metrics_collector/linux_cpu_measurement_node.cpp
  src/system_metrics_collector/linux_memory_measurement_node.cpp
  src/system_metrics_collector/linux_process_cpu_measurement_node.cpp
  src/system_metrics_collector/linux_process_memory_measurement_node.cpp
  src/system_metrics_collector/periodic_measurement_node.cpp
  src/system_metrics_collector/proc_cpu_data.cpp
  src/system_metrics_collector/proc_pid_cpu_data.cpp
  src/system_metrics_collector/utilities.cpp
)
ament_target_dependencies(system_metrics_collector
        message_filters
        metrics_statistics_msgs
        rcl
        rclcpp
        rclcpp_components
        rclcpp_lifecycle
        rcpputils
        rcutils
        libstatistics_collector)
rclcpp_components_register_nodes(system_metrics_collector
  system_metrics_collector::LinuxProcessCpuMeasurementNode)
rclcpp_components_register_nodes(system_metrics_collector
  system_metrics_collector::LinuxProcessMemoryMeasurementNode)
ament_export_libraries(system_metrics_collector)

add_executable(example_main src/system_metrics_collector/example_main.cpp)
target_link_libraries(example_main system_metrics_collector)
ament_target_dependencies(example_main)

add_executable(linux_cpu_collector src/system_metrics_collector/linux_cpu_collector.cpp)
target_link_libraries(linux_cpu_collector system_metrics_collector)
ament_target_dependencies(linux_cpu_collector)

add_executable(linux_memory_collector src/system_metrics_collector/linux_memory_collector.cpp)
target_link_libraries(linux_memory_collector system_metrics_collector)
ament_target_dependencies(linux_memory_collector)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  ament_lint_auto_find_test_dependencies()

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
  ament_target_dependencies(test_linux_memory_measurement_node lifecycle_msgs metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_process_cpu_measurement_node
          test/system_metrics_collector/test_linux_process_cpu_measurement_node.cpp)
  target_link_libraries(test_linux_process_cpu_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_process_cpu_measurement_node lifecycle_msgs metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_linux_process_memory_measurement_node
          test/system_metrics_collector/test_linux_process_memory_measurement_node.cpp)
  target_link_libraries(test_linux_process_memory_measurement_node system_metrics_collector)
  ament_target_dependencies(test_linux_process_memory_measurement_node lifecycle_msgs metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_periodic_measurement_node
    test/system_metrics_collector/test_periodic_measurement_node.cpp)
  target_link_libraries(test_periodic_measurement_node system_metrics_collector)
  ament_target_dependencies(test_periodic_measurement_node lifecycle_msgs metrics_statistics_msgs rclcpp)

  ament_add_gtest(test_composition
          test/system_metrics_collector/test_composition.cpp)
  ament_target_dependencies(test_composition class_loader rclcpp rclcpp_components)

  ament_add_gtest(test_utilities
          test/system_metrics_collector/test_utilities.cpp)
  target_link_libraries(test_utilities system_metrics_collector)
  ament_target_dependencies(test_utilities rclcpp)

  ament_add_gtest(test_subscriber_topic_statistics
          test/topic_statistics_collector/test_subscriber_topic_statistics.cpp)
  target_link_libraries(test_subscriber_topic_statistics system_metrics_collector)
  ament_target_dependencies(test_subscriber_topic_statistics rcl rclcpp sensor_msgs)
endif()

# Install launch files
install(DIRECTORY
  share/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}/
)

# Install entry points
install(TARGETS
  example_main
  DESTINATION
  lib/${PROJECT_NAME}
)

install(TARGETS
  linux_cpu_collector
  DESTINATION
  lib/${PROJECT_NAME}
)

install(TARGETS
  linux_memory_collector
  DESTINATION
  lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION
  lib
)

ament_export_libraries(${PROJECT_NAME})

ament_package()
