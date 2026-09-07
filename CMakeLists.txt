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

# ⚑ OPTIONAL SINCE 2026-09-07, AND THAT IS THE WHOLE PORT TO jazzy AND kilted.
# `find_package(nav2_ros_common REQUIRED)` on this line is what failed in about
# one second on both of them: the package first exists at nav2 1.5.0, and
# neither ros-jazzy-nav2-ros-common nor ros-kilted-nav2-ros-common is published
# at all. The package's own use of it turned out to be four type aliases and a
# QoS preset (see include/hubot/nav2_compat.hpp), so the requirement was ours,
# not the job's. QUIET, not REQUIRED, and the result is a MEASUREMENT: the
# availability of the package IS the fact the code below needs to know.
find_package(nav2_ros_common QUIET)
if(nav2_ros_common_FOUND)
  message(STATUS "hubot: nav2_ros_common present -- nav2 >= 1.5.0 line")
else()
  message(STATUS
    "hubot: nav2_ros_common absent -- pre-1.5.0 nav2 (jazzy 1.3.12 / kilted 1.4.2)")
endif()
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

include(CheckCXXSourceCompiles)

# ─────────────────────────────────────────────────────────────────────────────
# ⚑ TWO DISTRO PROBES, ADDED 2026-09-07. Each measures the fact it guards.
#
# They are NOT one probe with two consumers, and not a version comparison. This
# file already records why a version number cannot answer questions like these
# -- tag 1.5.1 and branch HEAD both declare "1.5.1" -- and two facts that happen
# to have changed in the same upstream release are still two facts.
#
# The compile probe below has a working NEGATIVE control, which is the only
# reason it is worth running: measured 2026-09-07 it returns 1 on jazzy
# (nav2_costmap_2d 1.3.12) and empty on released 1.5.1. A probe that cannot come
# back false has measured nothing.
# ─────────────────────────────────────────────────────────────────────────────
if(nav2_ros_common_FOUND)
  # Consumed by include/hubot/nav2_compat.hpp to pick the lifecycle-node class a
  # `Layer` will accept: nav2_util::LifecycleNode before 1.5.0, nav2::LifecycleNode
  # after. PUBLIC so the test targets, which host the filter, see the same answer
  # the filter does.
  target_compile_definitions(hubot_costmap_filters PUBLIC HUBOT_NAV2_HAS_ROS_COMMON=1)
  target_link_libraries(hubot_costmap_filters PUBLIC nav2_ros_common::nav2_ros_common)
endif()

# ⚑ THERE IS NO CMAKE PROBE FOR `CostmapFilter::worldToMask()`, AND THE REASON
# IS ON THE RECORD RATHER THAN OMITTED.
#
# One was written here first: a `check_cxx_source_compiles` deriving from
# CostmapFilter to reach the protected member. Its POSITIVE control -- merely
# `#include <nav2_costmap_2d/costmap_filters/costmap_filter.hpp>` and nothing
# else -- came back FALSE on all three distros, measured 2026-09-07, because
# `CMAKE_REQUIRED_INCLUDES` cannot be given that header's transitive include
# closure at configure time (it stopped at
# `rosidl_runtime_cpp/bounded_vector.hpp: No such file or directory`), and
# `CMAKE_REQUIRED_LIBRARIES` with the imported targets fails differently on
# CMake 4.2 with "No known features for C compiler". Every result that probe
# produced was FALSE FOR THE WRONG REASON -- including on jazzy, where the
# member is present -- and shipping it would have chosen the wrong branch on
# exactly the distros this port is for.
#
# The choice is made in src/zone_parameter_filter.cpp with `__has_include`
# instead, and that file explains why a header test is honest THERE and was not
# honest for the ZONE_PARAMETER_FILTER probe below: both of its branches are a
# real call that the compiler checks, so a wrong guess is a build error, not a
# silent wrong answer. That is the whole difference, and it is the reason the
# rule "probe the symbol, not the header" does not extend to this one.

# ⚑ DOES THIS nav2 CARRY ZONE_PARAMETER_FILTER? Probe the SYMBOL, not the header.
# `filter_values.hpp` exists at release tag 1.5.1 AND on branch HEAD; only the
# branch defines the constant inside it. So `__has_include` cannot tell them
# apart -- measured 2026-09-06, both substrates, file present in both -- and the
# first draft of this fix proposed exactly that and would have guarded nothing.
# The result drives a static_assert in the public header: where upstream DOES
# carry the constant, a value disagreement is a compile error rather than a
# silent wire mismatch; where it does not, hubot's own discriminator stands
# alone and the always-on BINARY_FILTER+1 cross-check still fires.
set(CMAKE_REQUIRED_INCLUDES ${nav2_costmap_2d_INCLUDE_DIRS})
set(CMAKE_REQUIRED_FLAGS "-std=c++17")
check_cxx_source_compiles("
#include <cstdint>
#include <nav2_costmap_2d/costmap_filters/filter_values.hpp>
int main() { return static_cast<int>(nav2_costmap_2d::ZONE_PARAMETER_FILTER); }
" HUBOT_UPSTREAM_HAS_ZONE_PARAMETER_FILTER)
unset(CMAKE_REQUIRED_INCLUDES)
unset(CMAKE_REQUIRED_FLAGS)
if(HUBOT_UPSTREAM_HAS_ZONE_PARAMETER_FILTER)
  # PUBLIC so every test target linking this library inherits the cross-check.
  target_compile_definitions(hubot_costmap_filters PUBLIC
    HUBOT_UPSTREAM_HAS_ZONE_PARAMETER_FILTER=1)
  message(STATUS "hubot: upstream nav2 carries ZONE_PARAMETER_FILTER -- our discriminator is cross-checked against it at compile time")
else()
  message(STATUS "hubot: upstream nav2 does NOT carry ZONE_PARAMETER_FILTER (release-era) -- using hubot::kZoneParameterFilterType alone")
endif()

install(TARGETS hubot_costmap_filters
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(FILES hubot_plugins.xml DESTINATION share/${PROJECT_NAME})
pluginlib_export_plugin_description_file(nav2_costmap_2d hubot_plugins.xml)

# ⚑ THE BRING-UP, ADDED 2026-09-06. Until this block existed the package shipped
# a plugin and told the integrator to build the other four things a costmap
# filter needs before it can run at all: a mask, a map_server publishing it, a
# costmap_filter_info_server, and params plus a launch file wiring them.
#
# These are INSTALLED, not merely committed. The falsifier for the whole block is
# that `ros2 launch hubot zone_filter_demo_launch.py` works from a clean-room
# container that has the installed package and NO file from this repository -- so
# anything the launch file reads has to arrive here or it does not exist.
add_executable(zone_target_demo_node src/zone_target_demo_node.cpp)
target_link_libraries(zone_target_demo_node
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS})
install(TARGETS zone_target_demo_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params maps DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_ros REQUIRED)

  set(hubot_test_deps
    hubot_costmap_filters
    nav2_costmap_2d::nav2_costmap_2d_core
    nav2_costmap_2d::filters
    nav2_util::nav2_util_core
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

  # ⚑ THE MAINTAINER-FINDING HARNESS. Every case drives a SEQUENCE and asserts
  # INSIDE a transient -- between a reset and the mask coming back, between a set
  # being issued and answered. The maintainer's findings all live in sequences;
  # a value oracle cannot fail on one, which is why a suite that passes every
  # value check can still catch none of them. (This said "486 passing tests"
  # until 2026-09-06. Nothing on this tree measures 486 anything, and no surface
  # says what it counted -- an unsourceable figure carrying an argument the
  # mechanism carries on its own. Removed rather than replaced with a guess.) Its fixture deliberately has NO wait-for-active after a reset,
  # because the helper that waits for one waits the defect out.
  ament_add_gtest(maintainer_findings_test
    test/maintainer_findings_test.cpp TIMEOUT 300)
  target_link_libraries(maintainer_findings_test ${hubot_test_deps})

  # ⚑ THE SOTIF CLASS ORACLE (FSE, 2026-09-06). Every other suite here asserts
  # an INSTANCE of the yes-when-nobody-is-watching family; this one asserts the
  # PROPERTY over the liveness gate's configuration space, because a suite that
  # adds one case per generation is always one generation behind.
  # ⚑ A RUN ON 2026-09-06 SAID SC-1, SC-3 AND SC-5 RED; SC-2, SC-4, SC-6
  # GREEN; 33 cases, 2 failures. THAT IS HISTORY, NOT THE STATE OF THIS TREE,
  # and it is kept rather than deleted because two things are wrong with it.
  # It names THREE red and counts TWO failures, so it did not agree with
  # itself on the day it was written. And all three named cases were worked
  # on AFTERWARDS -- see the CHANGELOG entries for the liveness fallback,
  # `declaredValidForS()` and the decision publisher's offered deadline --
  # with nothing coming back to update this comment. A comment that states a
  # test colour goes false the moment someone repairs what it measured.
  # The colours belong to a run. Read the run, not this line.
  # ⚑ This comment said SC-1/SC-3 "have never been run… the package cannot build
  # at all today". That was false when written and is kept in the record at
  # test/sotif_gate_inertness_test.cpp rather than deleted.
  ament_add_gtest(sotif_gate_inertness_test
    test/sotif_gate_inertness_test.cpp TIMEOUT 300)
  target_link_libraries(sotif_gate_inertness_test ${hubot_test_deps})

  # ⚑ THE NAMESPACED-HOST DISCRIMINATOR. The finding it answers was recorded as
  # visible only in a real multi-process stack and "structurally invisible to
  # every in-process fixture, which hosts the target at root namespace where a
  # relative name resolves correctly BY ACCIDENT". That was read as needing a
  # robot. It does not: the variable is where the HOST lives, not the target.
  # This fixture puts the filter's own node in `/local_costmap` and leaves the
  # target at the root, which is the one thing no other suite here does.
  # Cases A/B assert the resolved name and its arrival; C/D are a PAIR asserting
  # that the failure sentence tells an operator whether there is a node to walk
  # to at all -- remove the discrimination and one of the two goes red.
  ament_add_gtest(namespaced_target_resolution_test
    test/namespaced_target_resolution_test.cpp TIMEOUT 300)
  target_link_libraries(namespaced_target_resolution_test ${hubot_test_deps})

  # ⚑ THE STATIC INVARIANTS, WIRED IN 2026-09-06. It had been committed since
  # 2026-09-05 and referenced by NOTHING -- not this file, not a hook, not a
  # cron. It had therefore never fired, and when it was finally run by hand it
  # reported FAIL on 6 of 12 consecutive runs over unchanged source that
  # satisfies every invariant (a `| grep -q` under `set -o pipefail`: the early
  # exit SIGPIPEs the upstream grep and pipefail promotes the 141). A guard
  # nobody runs is not a guard, and a guard that is wrong half the time is worse
  # than none. Repaired, two-sided-controlled against two mutants, and now run
  # by `colcon test` where a regression is visible.
  #
  # ⚑ THIS REGISTRATION IS THE TOOLCHAIN-BOUND LANE, AND IT IS NOT THE ONLY ONE.
  # The script is deliberately toolchain-free (bash+grep+awk) precisely so it can
  # run where the behavioural suite cannot -- but this `add_test` is downstream of
  # sixteen `find_package(... REQUIRED)` calls, so on a ROS-free host cmake aborts
  # at :14 and never reaches this line: measured 2026-09-06, no CTestTestfile.cmake
  # is written at all. Registering it here therefore gives regression visibility to
  # people who already have ROS, and NOTHING to the host it was written for.
  # The toolchain-free lane is `bash test/safety_invariants_static.sh <dir>`,
  # documented in README.md where a reader who cannot build will actually reach it.
  # Do not delete that line from the README on the grounds that this one exists.
  add_test(NAME safety_invariants_static
    COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/safety_invariants_static.sh"
                 "${CMAKE_CURRENT_SOURCE_DIR}")

  # \u26d1 PROSE-AGAINST-TREE. Four figures about this package's own tests were
  # published at once and no two agreed; the README cited three `throw` lines that
  # had all moved; and a one-line summary in doc/SPEC_COVERAGE.md contradicted a
  # measurement printed twenty lines above it in the same file. Every one of them
  # was true when written. Nothing re-derived any of them, so each went false at
  # the next edit -- one of them twice in a single day.
  #
  # This re-derives them FROM THE TREE and fails when the prose disagrees. It also
  # carries FSE's two message oracles (SC-9 forbidden-token, SC-10 the
  # reassuring/degraded imperative asymmetry) because they read the same source.
  #
  # It ships its own negative controls: `python3 test/prose_matches_tree.py
  # --selftest` mutates a COPY of the tree once per check and requires each check
  # to go RED. A gate nobody has watched fail is not evidence.
  #
  # Same toolchain-bound caveat as the block above: this `add_test` is downstream
  # of sixteen `find_package(... REQUIRED)` calls, so on a ROS-free host cmake
  # never reaches it. The toolchain-free lane is `python3
  # test/prose_matches_tree.py`, which needs python3 and nothing else.
  add_test(NAME prose_matches_tree
    COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/prose_matches_tree.py"
                    "${CMAKE_CURRENT_SOURCE_DIR}")

  ament_add_gtest(survival_harness_test test/survival_harness_test.cpp TIMEOUT 300)
  target_link_libraries(survival_harness_test ${hubot_test_deps})
  target_compile_definitions(survival_harness_test PRIVATE
    ZPF_SURVIVAL_PROBE_PATH="$<TARGET_FILE:zpf_survival_probe>")
  add_dependencies(survival_harness_test zpf_survival_probe)
endif()

ament_package()
