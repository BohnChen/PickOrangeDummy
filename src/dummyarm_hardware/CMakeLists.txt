cmake_minimum_required(VERSION 3.16)
project(dummyarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-deprecated-declarations)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(dummyarm_hardware SHARED
  src/dummyarm_hardware.cpp
)
target_include_directories(dummyarm_hardware PRIVATE include)
ament_target_dependencies(dummyarm_hardware
  pluginlib hardware_interface rclcpp rclcpp_lifecycle std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface dummyarm_hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS dummyarm_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
