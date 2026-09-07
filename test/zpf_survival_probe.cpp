// Copyright (c) 2026 Komada (aki1770-del)
// SPDX-License-Identifier: Apache-2.0
//
// ⚑ THE SURVIVAL PROBE — a separate EXECUTABLE, on purpose.
//
// WHY (written before the act):
//
// The package's central claim is that it DEGRADES where upstream ABORTS. Upstream's
// abort is not "an exception is thrown"; it is THE PROCESS DIES:
//   checkPendingParameterUpdates() throws -> process() -> CostmapFilter::updateCosts()
//   (bare, costmap_filter.cpp:132) -> LayeredCostmap (no try/catch anywhere) -> the
//   costmap update THREAD's function -> std::terminate -> SIGABRT.
// The negation of that is PROCESS SURVIVAL, and survival is the thing an in-process
// oracle structurally cannot report:
//
//   * EXPECT_NO_THROW is only evaluated if control RETURNS to the assertion. abort(),
//     exit(), a terminate raised on another thread, or a deadlock all skip the oracle
//     entirely. An oracle skipped by the failure mode it exists to detect has measured
//     nothing.
//   * degrade_at_production_caller_test.cpp drives updateCosts() on the gtest MAIN
//     thread and spins the executors on that same thread. Production drives it on
//     Costmap2DROS's map-update thread while the executor spins elsewhere. The
//     exception that kills the node escapes THE COSTMAP THREAD — and that test has no
//     second thread at all, so it cannot produce the killing mechanism.
//   * A gtest death test cannot fix this. EXPECT_EXIT/EXPECT_DEATH use fork() WITHOUT
//     exec: fork clones only the calling thread, so every mutex another thread held is
//     held forever in the child. A death test asserting a CRASH survives that (its
//     child aborts before it needs those mutexes); a death test asserting SURVIVAL
//     needs the child to keep working, which is exactly what a forked child of a
//     threaded ROS process cannot do. Measured 2026-09-05: it hung for 3 minutes.
//
// So the process boundary must be created by exec, NOT by fork. This file is that
// child: a fresh process image, one thread at entry, building its own ROS graph.
//
// ⚑ NOTHING IN THIS FILE CATCHES AN EXCEPTION AROUND updateCosts(). That is not an
// oversight — it is the fidelity. LayeredCostmap has no handler, so neither do we.
// If the degrade path breaks, this process dies exactly the way the robot's does, and
// the PARENT (survival_harness_test.cpp) is alive to say so.
//
// CONTRACT WITH THE PARENT — exit code is the verdict; stdout carries the reason.
//   0  SURVIVED AND STILL WORKING (and only then is the beacon printed)
//   10 filter never became active            (setup failed; verdict is void, not green)
//   11 the costmap update thread STOPPED TURNING after the failure (wedged)
//   12 the rejection was never exercised — a vacuous run must not read as a pass
//   13 the read-only parameter was modified by the failed set
//   14 alive but NO LONGER FUNCTIONAL: a later, valid zone did not take effect
//   20 an exception reached main() during setup
// Any other outcome — a signal, or no exit at all — is the parent's to interpret, and
// is the whole reason the parent is a different process.

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hubot/nav2_compat.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";

// The parent greps for this. It is printed ONLY on the success path, after every
// check has passed, so a truncated or crashed run can never produce it.
constexpr char kBeacon[] = "ZPF_SURVIVAL_BEACON_OK";

std::string fp(const std::string & suffix)
{
  return std::string(kFilterName) + "." + suffix;
}

void addEntry(
  std::vector<rclcpp::Parameter> & cfg, const std::string & prefix,
  const std::string & target_node, const std::string & parameter,
  const rclcpp::ParameterValue & value)
{
  cfg.emplace_back(fp(prefix + ".node"), target_node);
  cfg.emplace_back(fp(prefix + ".parameter"), parameter);
  cfg.push_back(rclcpp::Parameter(fp(prefix + ".value"), value));
}

nav_msgs::msg::OccupancyGrid make_mask(uint32_t w, uint32_t h, int8_t fill_value)
{
  nav_msgs::msg::OccupancyGrid mask;
  mask.header.frame_id = "map";
  mask.info.resolution = 1.0;
  mask.info.width = w;
  mask.info.height = h;
  mask.info.origin.orientation.w = 1.0;
  mask.data.assign(w * h, fill_value);
  return mask;
}

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher()
  : Node("costmap_filter_info_pub")
  {
    publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      kInfoTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = hubot::kZoneParameterFilterType;
    msg->filter_mask_topic = kMaskTopic;
    msg->base = static_cast<float>(nav2_costmap_2d::BASE_DEFAULT);
    msg->multiplier = static_cast<float>(nav2_costmap_2d::MULTIPLIER_DEFAULT);
    publisher_->publish(std::move(msg));
  }
  ~InfoPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};

class MaskPublisher : public rclcpp::Node
{
public:
  explicit MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask)
  : Node("mask_pub")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }
  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

// `readonly_speed` is read-only, so a set against it is REJECTED by the target — this
// is upstream's own failing input, not a mock. `speed` is writable and is what the
// post-degrade liveness proof uses.
class TargetNode : public rclcpp::Node
{
public:
  TargetNode()
  : rclcpp::Node("zpf_target_node")
  {
    declare_parameter("speed", 1.0);
    rcl_interfaces::msg::ParameterDescriptor ro_desc;
    ro_desc.read_only = true;
    declare_parameter("readonly_speed", 1.0, ro_desc);
  }
};

}  // namespace

int main(int argc, char ** argv)
{
  // Mode selects which state map is loaded. Both drive the SAME production caller on
  // the SAME thread topology; only the configured zone differs.
  //   reject        : zone 1 sets a read-only parameter  -> the target REJECTS the set
  //   unknown-state : the mask carries an id no state declares
  const std::string mode = (argc > 1) ? argv[1] : "reject";

  // Line-buffered so the beacon and the reason lines reach the parent even if the
  // process is killed a moment later.
  setvbuf(stdout, nullptr, _IOLBF, 0);

  rclcpp::init(argc, argv);

  std::atomic<bool> spin_stop{false};
  std::atomic<bool> drive_stop{false};
  std::atomic<int> drive_cycles{0};
  int rc = 0;

  {
    rclcpp::executors::SingleThreadedExecutor node_executor;
    rclcpp::executors::SingleThreadedExecutor target_executor;
    rclcpp::executors::SingleThreadedExecutor pub_executor;

    auto target_node = std::make_shared<TargetNode>();
    target_executor.add_node(target_node);

    std::vector<rclcpp::Parameter> cfg;
    if (mode == "unknown-state") {
      // Zone 1 is declared; the mask will carry 7. applyState(7) finds no entry.
      cfg = {
        rclcpp::Parameter(fp("states"), std::vector<std::string>{"declared_zone"}),
        rclcpp::Parameter(fp("declared_zone.id"), 1),
        rclcpp::Parameter(fp("declared_zone.setpoints"), std::vector<std::string>{"sp"}),
      };
      addEntry(cfg, "declared_zone.sp", "zpf_target_node", "speed",
        rclcpp::ParameterValue(0.5));
    } else {
      cfg = {
        rclcpp::Parameter(fp("states"),
          std::vector<std::string>{"danger_zone", "recovery_zone"}),
        rclcpp::Parameter(fp("danger_zone.id"), 1),
        rclcpp::Parameter(fp("danger_zone.setpoints"), std::vector<std::string>{"ro_speed"}),
        rclcpp::Parameter(fp("recovery_zone.id"), 2),
        rclcpp::Parameter(fp("recovery_zone.setpoints"), std::vector<std::string>{"ok_speed"}),
      };
      addEntry(cfg, "danger_zone.ro_speed", "zpf_target_node", "readonly_speed",
        rclcpp::ParameterValue(0.5));
      addEntry(cfg, "recovery_zone.ok_speed", "zpf_target_node", "speed",
        rclcpp::ParameterValue(0.25));
    }

    std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
    };
    for (const auto & p : cfg) {
      overrides.push_back(p);
    }
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(overrides);

    std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers;
    std::shared_ptr<hubot::HostNode> node;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<hubot::ZoneParameterFilter> filter;
    std::shared_ptr<InfoPublisher> info_pub;
    std::shared_ptr<MaskPublisher> mask_pub;

    // Setup only. The drive loop below is deliberately NOT inside this try.
    try {
      node = std::make_shared<hubot::HostNode>("zpf_survival_host", "", opts);
      node_executor.add_node(node->get_node_base_interface());

      layers = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
      tf_buffer = hubot::createTransformBuffer(node);
      tf_buffer->setUsingDedicatedThread(true);

      filter = std::make_shared<hubot::ZoneParameterFilter>();
      filter->initialize(layers.get(), kFilterName, tf_buffer.get(), node, nullptr);
      filter->initializeFilter(kInfoTopic);

      info_pub = std::make_shared<InfoPublisher>();
      mask_pub = std::make_shared<MaskPublisher>(
        make_mask(4, 4, (mode == "unknown-state") ? 7 : 1));
      pub_executor.add_node(info_pub);
      pub_executor.add_node(mask_pub);
    } catch (const std::exception & ex) {
      printf("PROBE: exception during setup: %s\n", ex.what());
      rclcpp::shutdown();
      return 20;
    }

    // ── Thread S: the node's executor, exactly as a real node runs it. The async
    //    set_parameters future is completed HERE, on a different thread from the one
    //    that calls updateCosts() — the production topology, and the one the
    //    in-process test does not have.
    std::thread spinner([&]() {
        while (!spin_stop.load()) {
          pub_executor.spin_some();
          node_executor.spin_some();
          target_executor.spin_some();
          std::this_thread::sleep_for(5ms);
        }
      });

    auto wait_until = [&](auto pred, std::chrono::milliseconds timeout) {
        auto start = std::chrono::steady_clock::now();
        while (!pred()) {
          if (std::chrono::steady_clock::now() - start > timeout) {return false;}
          std::this_thread::sleep_for(10ms);
        }
        return true;
      };

    if (!wait_until([&]() {return filter->isActive();}, 8000ms)) {
      printf("PROBE: filter never became active\n");
      spin_stop = true;
      spinner.join();
      rclcpp::shutdown();
      return 10;
    }

    // ── Thread U: the costmap update thread. BARE. No handler, because
    //    LayeredCostmap::updateMap has none either. If anything escapes
    //    updateCosts(), std::terminate fires here and this process dies —
    //    which is precisely the event we are here to observe the ABSENCE of.
    std::thread updater([&]() {
        nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
        while (!drive_stop.load()) {
          filter->updateBounds(1.5, 1.5, 0.0, nullptr, nullptr, nullptr, nullptr);
          filter->updateCosts(costmap, 0, 0, 4, 4);
          drive_cycles.fetch_add(1);
          std::this_thread::sleep_for(20ms);
        }
      });

    if (mode == "unknown-state") {
      // Give the unknown state ample time to be sampled and applied.
      std::this_thread::sleep_for(2000ms);
      drive_stop = true;
      updater.join();
      printf("PROBE: unknown-state survived %d cycles, degraded=%d\n",
        drive_cycles.load(), static_cast<int>(filter->enforcementDegraded()));
      printf("%s\n", kBeacon);
      spin_stop = true;
      spinner.join();
      filter.reset();
      rclcpp::shutdown();
      return 0;
    }

    // ── 1. The rejection must actually happen. A run in which nothing was rejected
    //       is vacuous, and a vacuous run must never read as a pass.
    const bool degraded = wait_until(
      [&]() {return filter->enforcementDegraded();}, 8000ms);

    // ── 2. STILL FUNCTIONAL, not merely un-crashed. Drive the robot into a second,
    //       VALID zone and require the parameter to actually change on the target.
    //       Survival that cannot still enforce a zone is not the thing we claim.
    pub_executor.remove_node(mask_pub);
    mask_pub.reset();
    mask_pub = std::make_shared<MaskPublisher>(make_mask(4, 4, 2));
    pub_executor.add_node(mask_pub);

    const bool recovered = wait_until(
      [&]() {
        return target_node->get_parameter("speed").as_double() == 0.25;
      }, 8000ms);

    const double ro_after = target_node->get_parameter("readonly_speed").as_double();

    // ── 3. The costmap update thread must still be TURNING after the failure.
    //       An absolute cycle count would only say it once turned; what kills the
    //       robot is the loop that stops. So sample the counter, wait, sample again,
    //       and require it to have ADVANCED. A wedged update thread and a dead one
    //       are the same thing to her.
    const int cycles_before_window = drive_cycles.load();
    std::this_thread::sleep_for(500ms);
    const int cycles_after_window = drive_cycles.load();
    const int advanced = cycles_after_window - cycles_before_window;

    drive_stop = true;
    updater.join();
    const int cycles = drive_cycles.load();

    spin_stop = true;
    spinner.join();

    printf("PROBE: cycles=%d advanced_after_failure=%d degraded=%d recovered=%d "
      "readonly_speed=%.3f\n",
      cycles, advanced, static_cast<int>(degraded), static_cast<int>(recovered),
      ro_after);

    // 500ms at 20ms/cycle is ~25; 10 is a wide margin against a slow machine while
    // still being zero if the thread has stopped.
    if (advanced < 10) {
      printf("PROBE: the costmap update thread STOPPED TURNING after the failure "
        "(advanced %d in 500ms)\n", advanced);
      rc = 11;
    } else if (!degraded) {
      printf("PROBE: the rejection was never exercised -- vacuous run\n");
      rc = 12;
    } else if (ro_after != 1.0) {
      printf("PROBE: the read-only parameter was modified\n");
      rc = 13;
    } else if (!recovered) {
      printf("PROBE: alive but NO LONGER FUNCTIONAL -- a later valid zone did not "
        "take effect\n");
      rc = 14;
    } else {
      printf("%s\n", kBeacon);
      rc = 0;
    }

    filter.reset();
  }

  rclcpp::shutdown();
  return rc;
}
