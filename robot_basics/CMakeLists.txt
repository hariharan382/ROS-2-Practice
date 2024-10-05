cmake_minimum_required(VERSION 3.8)
project(robot_basics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  rclcpp
  std_msgs
)

add_executable(logger src/logger.cpp)
ament_target_dependencies(logger ${dependencies})

add_executable(logger_class src/logger_class.cpp)
ament_target_dependencies(logger_class ${dependencies})

add_executable(publisher_class src/publisher_class.cpp)
ament_target_dependencies(publisher_class ${dependencies})

add_executable(subscriber_class src/subscriber_class.cpp)
ament_target_dependencies(subscriber_class ${dependencies})

add_executable(param_reader src/param_reader.cpp)
ament_target_dependencies(param_reader ${dependencies})

add_executable(executors src/executors.cpp)
ament_target_dependencies(executors ${dependencies})

install(TARGETS
  logger
  logger_class
  publisher_class
  subscriber_class
  param_reader
  executors
  DESTINATION lib/${PROJECT_NAME}
)

#launch file 
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

If(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
