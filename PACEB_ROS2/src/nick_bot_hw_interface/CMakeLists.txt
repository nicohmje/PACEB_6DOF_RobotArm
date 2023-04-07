cmake_minimum_required(VERSION 3.8)
project(nick_bot_hw_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(dynamixel_workbench_toolbox REQUIRED)


add_library(${PROJECT_NAME} SHARED src/nick_bot_hw_interface.cpp src/arduino_comm.cpp )

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  controller_manager
  lifecycle_msgs
  serial
  rclcpp
  rclcpp_lifecycle
  pluginlib
  dynamixel_workbench_toolbox
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "NICK_BOT_HW_INTERFACE_BUILDING_DLL")


pluginlib_export_plugin_description_file(hardware_interface nick_bot_hw_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()


## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  dynamixel_workbench_toolbox
  serial
  controller_manager
  rclcpp
  rclcpp_lifecycle
)
ament_package()
