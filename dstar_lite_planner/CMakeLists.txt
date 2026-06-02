cmake_minimum_required(VERSION 3.8)
project(dstar_lite_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(DEPS
  rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d
  pluginlib nav_msgs geometry_msgs tf2_ros)

add_library(${PROJECT_NAME} SHARED
  src/field.cpp
  src/heuristic_functions.cpp
  src/dstar_lite.cpp
  src/dstar_lite_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${DEPS})

pluginlib_export_plugin_description_file(nav2_core dstar_lite_planner_plugin.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES dstar_lite_planner_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_package()
