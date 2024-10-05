cmake_minimum_required(VERSION 3.8)
project(robot_tf2_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
  sensor_msgs
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/robot_tf2_detector/ObstacleDetectorNode.cpp
    src/robot_tf2_detector/ObstacleMonitorNode.cpp
    src/robot_tf2_detector/ObstacleDetectorImprovedNode.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})


#executables
add_executable(detector src/detector_main.cpp)
ament_target_dependencies(detector ${dependencies})
target_link_libraries(detector ${PROJECT_NAME})

# add_executable(detector_improved src/detector_improved_main.cpp)
# ament_target_dependencies(detector_improved ${dependencies})
# target_link_libraries(detector_improved ${PROJECT_MAIN})


install(TARGETS
  ${PROJECT_NAME}
  detector
  # detector_improved
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
