cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_publisher)

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

add_executable(publisher_lambda lambda.cpp)
ament_target_dependencies(publisher_lambda rclcpp std_msgs)

add_executable(publisher_member_function member_function.cpp)
ament_target_dependencies(publisher_member_function rclcpp std_msgs)

add_executable(publisher_not_composable not_composable.cpp)
ament_target_dependencies(publisher_not_composable rclcpp std_msgs)

install(TARGETS
  publisher_lambda
  publisher_member_function
  publisher_not_composable
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

######################
## Project analysis ##
######################



# Addinional options for project analysis
set(SONARQUBE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package source directories for SonarQube.")
set(TEST_COVERAGE_PACKAGES_FILE "none" CACHE STRING "Path to the file that contains the package names to generate coverage reports for.")
OPTION(TEST_COVERAGE "Generate test coverage reports for SonarQube upon testing." OFF)

if(NOT ${SONARQUBE_PACKAGES_FILE} MATCHES "none")
  file(APPEND ${SONARQUBE_PACKAGES_FILE} "${PROJECT_NAME};${PROJECT_SOURCE_DIR}\n")
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
			              TARGETS publisher_lambda)
endif()
