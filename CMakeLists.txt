# hubot build file.
#
# This replaced an older CMake written against a removed ament idiom
# (`ament_target_dependencies`), which made the package fail at CONFIGURE --
# before a compiler ever ran. Dependencies are linked here as exported targets
# instead, which is what nav2_costmap_2d itself does on this branch.
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

  # Loads the filter the way nav2 does -- by pluginlib class name out of the
  # ament index, not by dlopen -- and drives it through a live
  # LayeredCostmap::updateMap(). Carries a negative control that requires the
  # UPSTREAM filter to throw on the same input through the same harness.
  ament_add_gtest(pluginlib_live_costmap_test
    test/pluginlib_live_costmap_test.cpp TIMEOUT 300)
  target_link_libraries(pluginlib_live_costmap_test ${hubot_test_deps})

  # ⚑ THE SURVIVAL PROBE — a separate EXECUTABLE, not a gtest target.
  # The package claims it DEGRADES where upstream ABORTS; upstream's abort IS
  # process death, so the oracle must OUTLIVE the failure. An in-process
  # EXPECT_NO_THROW is skipped by abort/exit/terminate-on-another-thread/deadlock,
  # and a gtest death test cannot help because it forks WITHOUT exec (fork clones
  # only the calling thread; a child asserting SURVIVAL then deadlocks on a mutex
  # another thread held -- measured, 3-minute hang, 2026-09-05).
  # fork+execv gives the process boundary that fork alone does not.
  add_executable(zpf_survival_probe test/zpf_survival_probe.cpp)
  target_link_libraries(zpf_survival_probe ${hubot_test_deps})

  # ⚑ THE STOPPED-vs-QUIET ORACLE. Drives the filter from the test thread while
  # the executor spins the node on ANOTHER thread -- production's topology
  # (controller_server.cpp:72) -- so that stopping the costmap update loop does
  # not stop the executor. Two arms differing in exactly one thing, plus TWO
  # negative controls that require the ORIGINAL DEFECT to reappear when the
  # mechanism is switched off.
  ament_add_gtest(costmap_silence_liveness_test
    test/costmap_silence_liveness_test.cpp TIMEOUT 300)
  target_link_libraries(costmap_silence_liveness_test ${hubot_test_deps})

  ament_add_gtest(survival_harness_test test/survival_harness_test.cpp TIMEOUT 300)
  target_link_libraries(survival_harness_test ${hubot_test_deps})
  target_compile_definitions(survival_harness_test PRIVATE
    ZPF_SURVIVAL_PROBE_PATH="$<TARGET_FILE:zpf_survival_probe>")
  add_dependencies(survival_harness_test zpf_survival_probe)
endif()

ament_package()
