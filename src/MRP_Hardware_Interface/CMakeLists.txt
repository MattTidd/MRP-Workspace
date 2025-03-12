cmake_minimum_required(VERSION 3.16)
project(MRP_Hardware_Interface LANGUAGES CXX)

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
  MRP_Hardware_Interface
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(MRP_Hardware_Interface PUBLIC cxx_std_17)
target_include_directories(MRP_Hardware_Interface PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/MRP_Hardware_Interface>
)
ament_target_dependencies(
  MRP_Hardware_Interface PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(MRP_Hardware_Interface PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface MRP_Hardware_Interface.xml)

# INSTALL
install(
  DIRECTORY bringup description hardware    # these are the directories I need
  DESTINATION share/${PROJECT_NAME}         # this is the destination 
)

install(TARGETS MRP_Hardware_Interface
  EXPORT export_MRP_Hardware_Interface
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
ament_export_targets(export_MRP_Hardware_Interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
