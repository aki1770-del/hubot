# hubot_live_stack — build file.
#
# Linked as exported targets rather than `ament_target_dependencies`, which was
# removed in ament_cmake 2.8.8 and is what made `hubot` itself fail at CONFIGURE
# before a compiler ever ran (outputs/fbr/hubot_clean_build_reproduction_2026_09_05.md).
cmake_minimum_required(VERSION 3.10)
project(hubot_live_stack)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

set(common_deps
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  ${rcl_interfaces_TARGETS})

# The bounds-declaring layer, loaded BY the real Costmap2DROS THROUGH pluginlib.
# Without it the filter is never driven at all -- see the file header.
add_library(hubot_live_stack_layers SHARED src/full_window_bounds_layer.cpp)
target_link_libraries(hubot_live_stack_layers PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  pluginlib::pluginlib
  rclcpp::rclcpp)
pluginlib_export_plugin_description_file(nav2_costmap_2d hubot_live_stack_plugins.xml)

# (3) THE PARAMETER TARGET, in its own OS process.
add_executable(param_target_node src/param_target_node.cpp)
target_link_libraries(param_target_node ${common_deps})

# The costmap-side preconditions (filter info + mask + continuous TF), also its
# own process so that withdrawing one is a real event on the wire.
add_executable(scenario_support_node src/scenario_support_node.cpp)
target_link_libraries(scenario_support_node
  ${common_deps}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros)

# (4) THE REAL BEHAVIOUR-TREE RECOVERY. Links behaviortree_cpp and
# nav2_ros_common only; the nav2 BT node itself is loaded at RUNTIME from
# nav2_behavior_tree's own installed plugin, so nothing here reimplements it.
add_executable(bt_clear_costmap_driver src/bt_clear_costmap_driver.cpp)
target_link_libraries(bt_clear_costmap_driver
  ${common_deps}
  ${nav2_msgs_TARGETS}
  nav2_ros_common::nav2_ros_common
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp)

# The observer, outside every process it observes.
add_executable(decision_observer src/decision_observer.cpp)
target_link_libraries(decision_observer
  ${common_deps}
  ${diagnostic_msgs_TARGETS})

install(TARGETS hubot_live_stack_layers
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(FILES hubot_live_stack_plugins.xml DESTINATION share/${PROJECT_NAME})

install(TARGETS
  param_target_node
  scenario_support_node
  bt_clear_costmap_driver
  decision_observer
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config scripts DESTINATION share/${PROJECT_NAME})

ament_package()
