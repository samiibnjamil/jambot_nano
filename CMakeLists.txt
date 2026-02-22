cmake_minimum_required(VERSION 3.16)
project(jambot_nano LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  hardware_interface
  controller_manager
  rclcpp_lifecycle
  pluginlib
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  jambot_nano
  SHARED
  src/hardware_interface/jambot_system.cpp
  src/hardware_interface/arduino_comms.cpp
  src/hardware_interface/wheel.cpp
)
target_compile_features(jambot_nano PUBLIC cxx_std_17)
target_include_directories(jambot_nano PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/jambot_nano>
)
ament_target_dependencies(
  jambot_nano PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(jambot_nano PUBLIC serial)

# Add controller node executable
add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(controller_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

add_executable(battery_state_publisher src/battery_state_publisher.cpp)
target_include_directories(battery_state_publisher PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(battery_state_publisher
  control_msgs
  rclcpp
  sensor_msgs
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "JAMBOT_NANO_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface jambot_nano.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/jambot_nano
)

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/jambot_nano
)

install(
  FILES jambot_nano.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS jambot_nano
  EXPORT export_jambot_nano
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install controller node
install(TARGETS controller_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS battery_state_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_targets(export_jambot_nano HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
