cmake_minimum_required(VERSION 3.8)
project(robot_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(control_msgs REQUIRED)
find_package(robot_tracking_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  robot_tracking_msgs
  sensor_msgs
  geometry_msgs
  vision_msgs
  control_msgs
  image_transport
  cv_bridge
  OpenCV
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/robot_tracking/ObjectDetector.cpp
  src/robot_tracking/HeadController.cpp
  src/robot_tracking/PIDController.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(object_detector src/object_detector_main.cpp)
ament_target_dependencies(object_detector ${dependencies})
target_link_libraries(object_detector ${PROJECT_NAME})

add_executable(object_tracker src/object_tracker_main.cpp)
ament_target_dependencies(object_tracker ${dependencies})
target_link_libraries(object_tracker ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  object_detector
  object_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(tests)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
