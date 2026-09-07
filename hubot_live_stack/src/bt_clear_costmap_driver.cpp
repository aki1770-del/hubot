// hubot_live_stack — THE RECOVERY, THROUGH A REAL BEHAVIOUR TREE.
//
// hubot's own comments state the consequence this exists to produce:
//   degrade_at_production_caller_test.cpp:579  "the path is ORDINARY:
//     ClearEntireCostmap sits in seven of nav2's default behaviour trees and
//     reaches ... Costmap2DROS::resetLayers() (costmap_2d_ros.cpp:719)"
// and then reaches it by calling `filter_->resetFilter()` on the test thread
// (:601). Between that call and the real thing sit every part that can fail:
//
//   BT tick -> nav2_behavior_tree::ClearEntireCostmapService (a BtServiceNode)
//           -> nav2_msgs/srv/ClearEntireCostmap over the middleware
//           -> nav2_costmap_2d::ClearCostmapService (clear_costmap_service.cpp:63,
//              service name "clear_entirely_" + costmap name)
//           -> Costmap2DROS::resetLayers()            (costmap_2d_ros.cpp:719)
//           -> Layer::reset() on every filter          (:735)
//           -> CostmapFilter::reset() = resetFilter(); initializeFilter(...)
//
// and crucially it arrives on the COSTMAP NODE'S EXECUTOR THREAD while
// `map_update_thread_` may be inside `updateCosts()` holding the same mutex.
// A direct `resetFilter()` call from a test thread reproduces neither the
// service hop nor that contention.
//
// This loads the ACTUAL nav2 plugin `libnav2_clear_costmap_service_bt_node.so`
// into a real BT::BehaviorTreeFactory. Nothing here reimplements the BT node.
//
// SUBSTRATE ONLY: this program reports what the tick returned and exits with a
// code. It makes no claim about what the filter should have done.

#include <chrono>
#include <filesystem>
#include <cstdio>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::string service_name = "/local_costmap/clear_entirely_local_costmap";
  // Resolved through the ament index rather than hardcoded, so the harness runs
  // wherever nav2_behavior_tree is installed and not only in one image.
  std::string plugin_path;
  try {
    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix("nav2_behavior_tree", prefix);
    plugin_path = (prefix / "lib" / "libnav2_clear_costmap_service_bt_node.so").string();
  } catch (const std::exception & e) {
    fprintf(stderr, "hubot_live_stack: cannot locate nav2_behavior_tree: %s\n", e.what());
    return 2;
  }
  int ticks = 1;
  int server_timeout_ms = 10000;
  int wait_for_service_ms = 20000;

  for (int i = 1; i < argc; ++i) {
    const std::string a{argv[i]};
    if (a == "--service" && i + 1 < argc) {service_name = argv[++i];} else if (
      a == "--plugin" && i + 1 < argc) {plugin_path = argv[++i];} else if (
      a == "--ticks" && i + 1 < argc) {ticks = std::atoi(argv[++i]);} else if (
      a == "--server-timeout-ms" && i + 1 < argc) {server_timeout_ms = std::atoi(argv[++i]);} else if
    (a == "--wait-for-service-ms" && i + 1 < argc) {wait_for_service_ms = std::atoi(argv[++i]);}
  }

  // BtServiceNode pulls a `nav2::LifecycleNode::SharedPtr` off the blackboard
  // under "node" (bt_service_node.hpp:123). The type must match exactly or the
  // blackboard get() throws -- this is why the driver links nav2_ros_common
  // rather than using a plain rclcpp::Node.
  auto node = std::make_shared<nav2::LifecycleNode>("hubot_live_stack_bt_driver");
  auto spin_thread = std::thread(
    [node]() {
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(node->get_node_base_interface());
      exec.spin();
    });

  BT::BehaviorTreeFactory factory;
  try {
    factory.registerFromPlugin(plugin_path);
  } catch (const std::exception & e) {
    fprintf(stderr, "hubot_live_stack: failed to load BT plugin %s: %s\n",
      plugin_path.c_str(), e.what());
    rclcpp::shutdown();
    if (spin_thread.joinable()) {spin_thread.join();}
    return 2;
  }

  auto blackboard = BT::Blackboard::create();
  blackboard->set<nav2::LifecycleNode::SharedPtr>("node", node);
  blackboard->set<std::chrono::milliseconds>(
    "server_timeout", std::chrono::milliseconds(server_timeout_ms));
  blackboard->set<std::chrono::milliseconds>("bt_loop_duration", 10ms);
  blackboard->set<std::chrono::milliseconds>(
    "wait_for_service_timeout", std::chrono::milliseconds(wait_for_service_ms));
  blackboard->set<int>("number_recoveries", 0);

  // The node name mirrors nav2's own default trees, where this node is written
  // as <ClearEntireCostmap name="ClearLocalCostmap-Context" .../>.
  const std::string xml =
    R"(<root BTCPP_format="4">
  <BehaviorTree ID="HubotClearRecovery">
    <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name=")" + service_name +
    R"("/>
  </BehaviorTree>
</root>)";

  BT::NodeStatus status = BT::NodeStatus::IDLE;
  try {
    auto tree = factory.createTreeFromText(xml, blackboard);
    for (int i = 0; i < ticks; ++i) {
      status = tree.tickWhileRunning(10ms);
      printf(
        "hubot_live_stack: BT tick %d -> %s\n", i + 1, BT::toStr(status).c_str());
      fflush(stdout);
    }
  } catch (const std::exception & e) {
    fprintf(stderr, "hubot_live_stack: BT tree failed: %s\n", e.what());
    rclcpp::shutdown();
    if (spin_thread.joinable()) {spin_thread.join();}
    return 3;
  }

  rclcpp::shutdown();
  if (spin_thread.joinable()) {spin_thread.join();}
  return status == BT::NodeStatus::SUCCESS ? 0 : 1;
}
