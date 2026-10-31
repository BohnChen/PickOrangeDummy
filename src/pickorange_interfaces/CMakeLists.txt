cmake_minimum_required(VERSION 3.8)
project(pickorange_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/TargetArray.msg"
  "msg/ChassisState.msg"
  "msg/HazardReport.msg"
  "msg/RobotState.msg"
  "action/MoveToPose.action"
  "action/CutStem.action"
  "action/Creep.action"
  "srv/CheckReachable.srv"
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
