cmake_minimum_required(VERSION 3.16)
project(np_diffbot LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial
  kangaroo_x2_driver
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  np_diffbot
  SHARED
  hardware/np_diffbot_system.cpp
)

target_compile_features(np_diffbot PUBLIC cxx_std_17)
target_include_directories(
  np_diffbot 
  PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/np_diffbot>
)
ament_target_dependencies(
  np_diffbot PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "NP_DIFFBOT_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface np_diffbot.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/np_diffbot
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/np_diffbot
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/np_diffbot
)
install(TARGETS np_diffbot
  EXPORT export_np_diffbot
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(np_diffbot_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_np_diffbot_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_np_diffbot HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
