cmake_minimum_required(VERSION 3.5)
project(statistics_collector_lib)

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
find_package(rcl REQUIRED)
find_package(rcpputils REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/moving_average_statistics/moving_average.cpp
  src/moving_average_statistics/types.cpp
  src/collector/collector.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rcpputils)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_collector
    test/collector/test_collector.cpp)
  target_link_libraries(test_collector ${PROJECT_NAME})
  ament_target_dependencies(test_collector rcpputils)

  ament_add_gtest(test_moving_average_statistics
    test/moving_average_statistics/test_moving_average_statistics.cpp)
  target_link_libraries(test_moving_average_statistics ${PROJECT_NAME})


  ament_add_gtest(test_received_message_period
    test/topic_statistics_collector/test_received_message_period.cpp)
  target_link_libraries(test_received_message_period ${PROJECT_NAME})
  ament_target_dependencies(test_received_message_period rcl rcpputils)

  ament_add_gtest(test_received_message_age
    test/topic_statistics_collector/test_received_message_age.cpp)
  target_link_libraries(test_received_message_age ${PROJECT_NAME})
  ament_target_dependencies(test_received_message_age rcl rcpputils std_msgs sensor_msgs)
endif()

install(
  TARGETS "${PROJECT_NAME}"
  EXPORT "${PROJECT_NAME}-targets"
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rcl)
ament_export_dependencies(rcpputils)

ament_package()

export(TARGETS ${PROJECT_NAME}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake")
export(PACKAGE ${PROJECT_NAME})