cmake_minimum_required(VERSION 3.8)
project(rascl_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(soem REQUIRED)

add_library(
  rascl_hardware_interface
  SHARED
  src/rascl_hardware_interface.cpp
)

target_include_directories(
  rascl_hardware_interface
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(
  rascl_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(
  rascl_hardware_interface
  soem
)

pluginlib_export_plugin_description_file(
  hardware_interface
  rascl_hardware_interface.xml
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rascl_hardware_interface
  EXPORT export_rascl_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)

  ament_add_gtest(test_generic_system test/test_generic_system.cpp)
  target_link_libraries(test_generic_system rascl_hardware_interface)
  target_compile_definitions(
    test_generic_system
    PRIVATE
    RASCL_TEST_URDF_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../rascl_description/urdf/rascl.urdf"
  )

  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS "-legal/copyright")
endif()

ament_export_include_directories(include)
ament_export_libraries(rascl_hardware_interface)
ament_export_targets(export_rascl_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  soem
)

ament_package()
