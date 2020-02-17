cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS})


add_executable(subscriber_lambda src/lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp rclcpp_lifecycle std_msgs)

add_executable(subscriber_member_function src/member_function.cpp)
ament_target_dependencies(subscriber_member_function rclcpp std_msgs)

add_executable(subscriber_not_composable src/not_composable.cpp)
ament_target_dependencies(subscriber_not_composable rclcpp std_msgs)

if(BUILD_TESTING)

  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  
  add_launch_test(
    "test/test_subscriber.test.py"
    TARGET subscriber_lambda
    WORKING_DIRECTORY "$<TARGET_FILE_DIR:subscriber_lambda>"
    TIMEOUT 10
  )
endif()

install(TARGETS
  subscriber_lambda
  subscriber_member_function
  subscriber_not_composable
  DESTINATION lib/${PROJECT_NAME})

ament_package()


######################
## Project analysis ##
######################



# Addinional options for project analysis
set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
set(TEST_COVERAGE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package names to generate coverage reports for.")
OPTION(TEST_COVERAGE "Generate test coverage reports for SonarQube upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME}\n")
  message(${SONARQUBE_PACKAGES_FILE})
endif()

if(NOT ${TEST_COVERAGE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${TEST_COVERAGE_PACKAGES_FILE} "${PROJECT_NAME} ")
endif()

if(TEST_COVERAGE)
  # Set build type to debug to avoid compiler optimization when checking coverage
  set(CMAKE_BUILD_TYPE Debug)
  # Include coverage report related functions
  list(APPEND CMAKE_MODULE_PATH "/usr/lib/cmake/CodeCoverage")
  include(CodeCoverage)
  # Append compiler flags to gcc for generating coverage notes
  APPEND_COVERAGE_COMPILER_FLAGS()
  # Set coverage output dir to ${CMAKE_BINARY_DIR}/test_coverage
  SET_COVERAGE_OUTPUT_DIR()
endif()

if(TEST_COVERAGE)
  # Define a coverage target with some libs and executables to check
  #	NAME 	Name of coverage target. Default: coverage. Only works with default currently.
  #	TARGETS Library and executable targets for which coverage reports are requested.
  add_coverage_gcov(NAME coverage
			              TARGETS subscriber_lambda)
endif()
