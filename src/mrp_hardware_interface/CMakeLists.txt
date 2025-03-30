cmake_minimum_required(VERSION 3.16)
project(mrp_hardware_interface LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  mrp_hardware_interface
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(mrp_hardware_interface PUBLIC cxx_std_17)
target_include_directories(mrp_hardware_interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/mrp_hardware_interface>
)
ament_target_dependencies(
  mrp_hardware_interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(mrp_hardware_interface PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mrp_hardware_interface.xml)

# INSTALL
install(
  DIRECTORY hardware                        # these are the directories I need
  DESTINATION share/${PROJECT_NAME}         # this is the destination 
)

install(TARGETS mrp_hardware_interface
  EXPORT export_mrp_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# if(BUILD_TESTING)
#  find_package(ament_cmake_pytest REQUIRED)
#
#  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
#  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
#  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
# endif()

## EXPORTS
ament_export_targets(export_mrp_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
