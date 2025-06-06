cmake_minimum_required(VERSION 3.8)
project(sequence_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
include_directories(include)
add_executable(sequence_controller src/sequence_controller.cpp)

ament_target_dependencies(sequence_controller
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  sequence_controller
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
