cmake_minimum_required(VERSION 3.8)
project(template_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories()

# generate parameter listener hpp file
generate_parameter_library(template_ros2_controller_parameters
  src/template_ros2_controller_parameters.yaml
)

pluginlib_export_plugin_description_file(controller_interface template_ros2_controller_plugin.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/template_ros2_controller.cpp
)

target_link_libraries(template_ros2_controller template_ros2_controller_parameters)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
