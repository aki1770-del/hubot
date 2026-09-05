# ⚑ FBR TRIAGE PROBE — NOT hubot's committed CMakeLists.txt.
# Scratchpad-only. Purpose: get past the CONFIGURE failure at the committed
# file's line 31 (`ament_target_dependencies` removed in this ament era) so the
# blockers BEHIND it can be enumerated in one pass. This proves nothing about
# the committed tree.
cmake_minimum_required(VERSION 3.10)
project(hubot)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(tf2 REQUIRED)

add_library(hubot_costmap_filters SHARED src/zone_parameter_filter.cpp)
target_include_directories(hubot_costmap_filters PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(hubot_costmap_filters PUBLIC
  nav2_costmap_2d::nav2_costmap_2d_core
  nav2_costmap_2d::filters
  nav2_util::nav2_util_core
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  pluginlib::pluginlib
  tf2::tf2
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${rcl_interfaces_TARGETS})

install(TARGETS hubot_costmap_filters
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES hubot_plugins.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_costmap_2d hubot_plugins.xml)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_ros REQUIRED)

  set(hubot_test_deps
    hubot_costmap_filters
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_costmap_2d::filters
    nav2_util::nav2_util_core
    nav2_ros_common::nav2_ros_common
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    pluginlib::pluginlib
    tf2::tf2
    tf2_ros::tf2_ros
    ${std_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${nav2_msgs_TARGETS}
    ${diagnostic_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${rcl_interfaces_TARGETS})

  ament_add_gtest(test_degrade_not_abort test/test_degrade_not_abort.cpp)
  target_link_libraries(test_degrade_not_abort ${hubot_test_deps})

  # ⚑ Drives CostmapFilter::updateCosts() -- the caller production uses, and the
  # one upstream's own 770-line suite never calls once.
  ament_add_gtest(degrade_at_production_caller_test
    test/degrade_at_production_caller_test.cpp TIMEOUT 180)
  target_link_libraries(degrade_at_production_caller_test ${hubot_test_deps})

  # THE SURVIVAL PROBE -- a separate EXECUTABLE, not a gtest target.
  # fork+execv gives the process boundary that fork alone does not.
  add_executable(zpf_survival_probe test/zpf_survival_probe.cpp)
  target_link_libraries(zpf_survival_probe ${hubot_test_deps})

  ament_add_gtest(survival_harness_test test/survival_harness_test.cpp TIMEOUT 300)
  target_link_libraries(survival_harness_test ${hubot_test_deps})
  target_compile_definitions(survival_harness_test PRIVATE
    ZPF_SURVIVAL_PROBE_PATH="$<TARGET_FILE:zpf_survival_probe>")
  add_dependencies(survival_harness_test zpf_survival_probe)
endif()

ament_package()
