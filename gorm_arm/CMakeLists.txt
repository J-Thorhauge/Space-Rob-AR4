cmake_minimum_required(VERSION 3.5)
project(gorm_arm)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  cv_bridge
  OpenCV
)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(gorm_arm SHARED 
  src/gorm_arm_hardware_interface.cpp
  src/gorm_arm_gripper_hw_interface.cpp
  src/teensy_driver.cpp
  src/arduino_nano_driver.cpp
)

target_compile_features(gorm_arm PUBLIC cxx_std_17)
target_include_directories(gorm_arm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gorm_arm
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

# Add Arduino nano
add_executable(run_arduino_nano src/run_arduino_nano.cpp)

target_link_libraries(
  run_arduino_nano
  gorm_arm
)

install(TARGETS
  run_arduino_nano
  DESTINATION lib/${PROJECT_NAME}
)

# Add gripper interface node
add_executable(gripper_interface_node src/gripper_interface_node.cpp)

target_link_libraries(
  gripper_interface_node
  serial
  gorm_arm
)

install(TARGETS
  gripper_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add camera interface node
add_executable(camera_interface_node src/camera_interface_node.cpp)

target_link_libraries(
  camera_interface_node
  sensor_msgs
  cv_bridge
  OpenCV
  gorm_arm
)

install(TARGETS
  camera_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

install(TARGETS gorm_arm
  EXPORT export_gorm_arm
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_gorm_arm HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)

install(DIRECTORY config launch rviz srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
