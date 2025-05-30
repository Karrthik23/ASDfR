cmake_minimum_required(VERSION 3.8)
project(ball_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# include directories
include_directories(include)

# executable
add_executable(ball_detector src/detect_ball.cpp)

# link dependencies
ament_target_dependencies(ball_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# install
install(TARGETS
  ball_detector
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
