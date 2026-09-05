cmake_minimum_required(VERSION 3.8)
project(hubot)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(dependencies
  rclcpp rclcpp_lifecycle pluginlib nav2_costmap_2d nav2_msgs
  nav_msgs std_msgs diagnostic_msgs geometry_msgs rcl_interfaces)

add_library(hubot_costmap_filters SHARED src/zone_parameter_filter.cpp)
target_include_directories(hubot_costmap_filters PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(hubot_costmap_filters ${dependencies})

install(TARGETS hubot_costmap_filters
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES hubot_plugins.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_degrade_not_abort test/test_degrade_not_abort.cpp)
  target_link_libraries(test_degrade_not_abort hubot_costmap_filters)
  ament_target_dependencies(test_degrade_not_abort ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(hubot_costmap_filters)
ament_export_dependencies(${dependencies})
pluginlib_export_plugin_description_file(nav2_costmap_2d hubot_plugins.xml)
ament_package()
