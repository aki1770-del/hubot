// Copyright (c) 2026 Komada (aki1770-del)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// WHAT THIS FILE IS FOR
//
// Every other test in this package builds the filter with `std::make_shared`
// and calls its methods directly. That is the right call made by the wrong
// caller. In a running robot nobody constructs this filter and nobody calls
// `updateCosts()`. `LayeredCostmap::updateMap()` does, from inside its own
// lock, on a `shared_ptr<Layer>` that pluginlib handed it after resolving a
// class NAME out of the ament index.
//
// Two things were therefore still unproven before this file existed:
//
//   1. That pluginlib can resolve `hubot::ZoneParameterFilter` at all.
//      A shared library can `dlopen()` cleanly -- no undefined symbols -- and
//      still be invisible to pluginlib, or declare a base class that does not
//      match the one nav2 asks for. `dlopen` success is not plugin loading.
//
//   2. That a rejected parameter set, arriving through the real
//      `LayeredCostmap::updateMap()` call chain, does not escape it.
//
// The chain reproduced here is nav2's own, read from the installed sources:
//
//   costmap_2d_ros.hpp:387   pluginlib::ClassLoader<Layer>
//                            {"nav2_costmap_2d", "nav2_costmap_2d::Layer"}
//   costmap_2d_ros.cpp:187   createSharedInstance(filter_types_[i])
//   costmap_2d_ros.cpp:192   layered_costmap_->addFilter(filter)
//   costmap_2d_ros.cpp:613   layered_costmap_->updateMap(x, y, yaw)
//
// `layered_costmap.cpp` contains ZERO `try` or `catch` tokens, and
// `Costmap2DROS::updateMap()` wraps its call in none either. An exception that
// leaves a costmap filter therefore leaves the costmap update thread with no
// handler above it anywhere.
//
// THE NEGATIVE CONTROL IS THE POINT
//
// A harness reporting "this filter survived updateMap()" proves nothing unless
// the same harness is shown to CATCH the failure it claims to rule out. So the
// third test below loads the upstream nav2 filter -- `costmap_plugins.xml:42`,
// same ClassLoader, same LayeredCostmap, same rejecting input -- and REQUIRES
// it to throw out of `updateMap()`. If that test ever passes quietly, the other
// two are measuring the harness rather than the code, and must not be quoted.
//
// That control does double duty: `CostmapFilter::updateBounds()` deliberately
// leaves the update window untouched, so a costmap holding only a filter and no
// bounds-providing layer computes an EMPTY window and never calls a single
// `updateCosts()`. A throw from the control is positive proof that the
// production caller really did arrive at the filter.
//
// HONEST BOUND, kept in the file rather than in a report nobody re-reads:
// this drives a live `LayeredCostmap` in-process. It is NOT a running
// `controller_server` and it is NOT a robot. Both remain unverified.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/exceptions.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";

// The two class NAMES, resolved by pluginlib from the ament index: this
// package's, declared in hubot_plugins.xml, and nav2's own.
constexpr char kHubotClass[] = "hubot::ZoneParameterFilter";
constexpr char kVanillaClass[] = "nav2_costmap_2d::ZoneParameterFilter";

std::string fp(const std::string & suffix)
{
  return std::string(kFilterName) + "." + suffix;
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
    msg->base = 0.0f;
    msg->multiplier = 1.0f;
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

// ⚑ THE HUMAN-DECISION SURFACE. Feature 2 is half the package's reason to
// exist, and until 2026-09-06 NOTHING in this file read it: every test here
// asserted only that the process did not die. Survival is the robot's half.
// `zone_decision` is the PERSON's half -- the level an operator tool renders
// and the sentence a person acts on -- and a suite that proves one and not the
// other has proved half the claim.
//
// This is the surface an integrator actually has: `enforcementDegraded()` is
// public and NOTHING IN NAV2 CALLS IT, so a C++ accessor no caller invokes is
// not a channel to a human. The DiagnosticArray is.
class DecisionSubscriber : public rclcpp::Node
{
public:
  DecisionSubscriber()
  : Node("zpf_decision_sub")
  {
    subscriber_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "zone_decision", rclcpp::QoS(50),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        for (const auto & st : msg->status) {
          last_level_ = st.level;
          last_message_ = st.message;
          for (const auto & kv : st.values) {last_[kv.key] = kv.value;}
          ++count_;
        }
      });
  }

  std::string value(const std::string & k) const
  {
    auto it = last_.find(k);
    return it == last_.end() ? std::string("<absent>") : it->second;
  }
  std::string lastMessage() const {return last_message_;}
  uint8_t lastLevel() const {return last_level_;}
  size_t count() const {return count_;}

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  std::map<std::string, std::string> last_;
  std::string last_message_;
  uint8_t last_level_{255};   // 255 = nothing ever arrived; never a valid level
  size_t count_{0};
};

// The node whose `readonly_speed` parameter CANNOT be set. That rejection is
// the input under test: upstream turns it into a throw, this package turns it
// into a latched, logged degrade.
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

// NOT a stand-in for the filter -- a stand-in for the static or obstacle layer
// that sits beside it on every real robot. `CostmapFilter::updateBounds()`
// leaves min_x/max_x untouched (its four bound parameters are commented out in
// nav2's source), so a costmap holding only a filter computes an empty update
// window and never calls `updateCosts()` at all. Production always has a layer
// supplying bounds. Without this, every test in this file would pass for the
// wrong reason.
class FullWindowBoundsLayer : public nav2_costmap_2d::Layer
{
public:
  void reset() override {}
  bool isClearable() override {return false;}
  void updateBounds(
    double, double, double,
    double * min_x, double * min_y, double * max_x, double * max_y) override
  {
    *min_x = 0.0; *min_y = 0.0; *max_x = 4.0; *max_y = 4.0;
  }
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override {}
};

// The rejecting configuration: route a value at `readonly_speed` on the target
// node. The set is guaranteed to be refused, which is the whole input.
std::vector<rclcpp::Parameter> rejectingConfig()
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
    rclcpp::Parameter(fp("transform_tolerance"), 0.5),
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"danger_zone"}),
    rclcpp::Parameter(fp("danger_zone.id"), 1),
    rclcpp::Parameter(fp("danger_zone.ro_speed.node"), std::string("zpf_target_node")),
    rclcpp::Parameter(fp("danger_zone.ro_speed.parameter"), std::string("readonly_speed")),
    rclcpp::Parameter(fp("danger_zone.ro_speed.value"), 0.5),
  };
  // This package names the list `setpoints`; upstream names it `overrides`.
  // Declaring both keeps ONE configuration valid against either class, so the
  // subject and its control cannot drift apart by accident.
  cfg.emplace_back(fp("danger_zone.setpoints"), std::vector<std::string>{"ro_speed"});
  cfg.emplace_back(fp("danger_zone.overrides"), std::vector<std::string>{"ro_speed"});
  return cfg;
}

}  // namespace

class PluginlibLiveCostmap : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_node_ = std::make_shared<TargetNode>();
    target_executor_.add_node(target_node_);
    // Subscribed BEFORE the filter exists, so no transition can be missed.
    decision_sub_ = std::make_shared<DecisionSubscriber>();
    target_executor_.add_node(decision_sub_);
  }

  void TearDown() override
  {
    filter_.reset();
    bounds_.reset();
    layers_.reset();
    info_pub_.reset();
    mask_pub_.reset();
    if (node_) {node_executor_.remove_node(node_->get_node_base_interface());}
    node_.reset();
    target_executor_.remove_node(decision_sub_);
    decision_sub_.reset();
    target_executor_.remove_node(target_node_);
    target_node_.reset();
  }

  // The whole production chain, parameterised only by the class name so that
  // the subject and its negative control cannot diverge.
  bool bringUp(const std::string & plugin_class)
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(rejectingConfig());
    node_ = std::make_shared<nav2::LifecycleNode>("zpf_pluginlib_host", opts);
    node_executor_.add_node(node_->get_node_base_interface());

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layers_->resizeMap(4, 4, 1.0, 0.0, 0.0);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);

    // (1) THE LOADER -- same construction as costmap_2d_ros.hpp:387.
    filter_ = loader_.createSharedInstance(plugin_class);
    if (!filter_) {return false;}

    // (2) The Layer lifecycle Costmap2DROS drives.
    //     `CostmapFilter::activate()` is what calls `initializeFilter()`.
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->activate();

    bounds_ = std::make_shared<FullWindowBoundsLayer>();
    bounds_->initialize(layers_.get(), "bounds", tf_buffer_.get(), node_, nullptr);

    // (3) The costmap owns them -- costmap_2d_ros.cpp:166 and :192.
    layers_->addPlugin(bounds_);
    layers_->addFilter(filter_);

    info_pub_ = std::make_shared<InfoPublisher>();
    mask_pub_ = std::make_shared<MaskPublisher>(make_mask(4, 4, 1));
    pub_executor_.add_node(info_pub_);
    pub_executor_.add_node(mask_pub_);

    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(5s)) {
      spinAll();
      if (filterReportsActive()) {return true;}
      std::this_thread::sleep_for(10ms);
    }
    // The upstream class exposes no isActive(); for it we settle on time and
    // let its own assertion (that it throws) decide. A control that failed to
    // receive its mask would not throw, and would fail loudly rather than pass.
    spinFor(500ms);
    return true;
  }

  bool filterReportsActive()
  {
    auto h = std::dynamic_pointer_cast<hubot::ZoneParameterFilter>(filter_);
    return h ? h->isActive() : false;
  }

  void spinAll()
  {
    pub_executor_.spin_some();
    node_executor_.spin_some();
    target_executor_.spin_some();
  }

  // Spin until the predicate holds or the budget runs out. Returns whether it held.
  bool pumpUntil(const std::function<bool()> & pred, std::chrono::milliseconds d)
  {
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(d)) {
      if (pred()) {return true;}
      spinAll();
      std::this_thread::sleep_for(10ms);
    }
    return pred();
  }

  void spinFor(std::chrono::milliseconds d)
  {
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(d)) {
      spinAll();
      std::this_thread::sleep_for(10ms);
    }
  }

  // THE PRODUCTION CALL: layered_costmap.cpp:136, reached from
  // costmap_2d_ros.cpp:613. Driven twice because the parameter set is
  // asynchronous -- the first pass issues it, the second observes the refusal.
  void driveUpdateMapTwice()
  {
    layers_->updateMap(1.5, 1.5, 0.0);
    spinFor(400ms);
    layers_->updateMap(1.5, 1.5, 0.0);
  }

  pluginlib::ClassLoader<nav2_costmap_2d::Layer> loader_{
    "nav2_costmap_2d", "nav2_costmap_2d::Layer"};
  std::shared_ptr<TargetNode> target_node_;
  std::shared_ptr<DecisionSubscriber> decision_sub_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  nav2::TransformBuffer::SharedPtr tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::Layer> filter_;
  std::shared_ptr<nav2_costmap_2d::Layer> bounds_;
  std::shared_ptr<InfoPublisher> info_pub_;
  std::shared_ptr<MaskPublisher> mask_pub_;
  rclcpp::executors::SingleThreadedExecutor node_executor_;
  rclcpp::executors::SingleThreadedExecutor target_executor_;
  rclcpp::executors::SingleThreadedExecutor pub_executor_;

  // ⚑ THE REASON, OR EMPTY. A and C discriminate this package against
  // UPSTREAM's `nav2_costmap_2d::ZoneParameterFilter`, loaded by pluginlib
  // name. A released nav2 does not ship that class (measured 2026-09-06 at tag
  // 1.5.1: declared by nothing on the install), so on a release those cases
  // have no control. They SKIP with this reason -- a skip that says why, never
  // a pass manufactured by the absence of the thing under test. Declaration is
  // checked first (cheap, no throw), then loadability by attempt, so the reason
  // is right on either failure shape pluginlib produces.
  std::string upstreamControlUnavailableReason()
  {
    const auto declared = loader_.getDeclaredClasses();
    if (std::find(declared.begin(), declared.end(), std::string(kVanillaClass)) ==
      declared.end())
    {
      return std::string(kVanillaClass) +
             " is not declared to pluginlib on this substrate (a released nav2 does not "
             "ship it)";
    }
    try {
      auto probe = loader_.createSharedInstance(kVanillaClass);
      (void)probe;
    } catch (const pluginlib::PluginlibException & ex) {
      return std::string(kVanillaClass) + " is declared but not loadable: " + ex.what();
    }
    return "";
  }
};

// A0 -- ⚑ OUR OWN CLASS THROUGH PLUGINLIB, ON WHATEVER SUBSTRATE THIS RUNS.
//      Added 2026-09-06 on FBR's finding: "pluginlib loadability of hubot's own
//      class on the release is UNVERIFIED by this suite. Yesterday's dlopen is
//      not pluginlib." Case A below asserts the same load but ALSO loads
//      upstream's class, and on a release it aborts on that arm before this
//      assertion can report. This case depends on nothing upstream and must be
//      green on every substrate the package claims.
TEST_F(PluginlibLiveCostmap, A0_PluginlibResolvesHubotsOwnClassOnThisSubstrate)
{
  const auto declared = loader_.getDeclaredClasses();
  ASSERT_NE(
    std::find(declared.begin(), declared.end(), std::string(kHubotClass)),
    declared.end())
    << "pluginlib cannot see hubot::ZoneParameterFilter in the ament index on "
       "this substrate. nav2 loads costmap filters only through this loader "
       "(costmap_2d_ros.hpp:387); a dlopen() says nothing about it.";

  std::shared_ptr<nav2_costmap_2d::Layer> instance;
  ASSERT_NO_THROW(instance = loader_.createSharedInstance(kHubotClass));
  ASSERT_NE(instance, nullptr);
  EXPECT_NE(
    std::dynamic_pointer_cast<nav2_costmap_2d::CostmapFilter>(instance), nullptr)
    << "the declared base class must really resolve to a CostmapFilter";
  EXPECT_NE(std::dynamic_pointer_cast<hubot::ZoneParameterFilter>(instance), nullptr);

  const std::string lib = loader_.getClassLibraryPath(kHubotClass);
  std::cout << "[identity] " << kHubotClass << " -> " << lib << std::endl;
  EXPECT_NE(lib.find("hubot"), std::string::npos)
    << "hubot::ZoneParameterFilter resolved to a library that is not ours: " << lib;
}

// A -- pluginlib, not dlopen -- AND identity against upstream's class.
TEST_F(PluginlibLiveCostmap, A_PluginlibResolvesAndInstantiatesTheClass)
{
  if (const std::string why = upstreamControlUnavailableReason(); !why.empty()) {
    GTEST_SKIP() << "no upstream control on this substrate -- " << why
                 << ". This case discriminates hubot's class against upstream's and "
                    "cannot run without both; A0 above carries the hubot-only "
                    "assertions and must be green here.";
  }
  const auto declared = loader_.getDeclaredClasses();
  EXPECT_NE(
    std::find(declared.begin(), declared.end(), std::string(kHubotClass)),
    declared.end())
    << "pluginlib cannot see hubot::ZoneParameterFilter in the ament index. "
       "A successful dlopen() says nothing about this: nav2 loads costmap "
       "filters only through this loader (costmap_2d_ros.hpp:387).";

  std::shared_ptr<nav2_costmap_2d::Layer> instance;
  ASSERT_NO_THROW(instance = loader_.createSharedInstance(kHubotClass));
  ASSERT_NE(instance, nullptr);

  EXPECT_NE(
    std::dynamic_pointer_cast<nav2_costmap_2d::CostmapFilter>(instance), nullptr)
    << "the declared base class must really resolve to a CostmapFilter, or "
       "addFilter() would accept an object updateMap() cannot drive";
  EXPECT_NE(std::dynamic_pointer_cast<hubot::ZoneParameterFilter>(instance), nullptr);

  // Identity, not just loadability. The two class names must resolve to two
  // DIFFERENT shared libraries, or the negative control below is comparing this
  // package against itself and cannot discriminate anything.
  const std::string hubot_lib = loader_.getClassLibraryPath(kHubotClass);
  const std::string vanilla_lib = loader_.getClassLibraryPath(kVanillaClass);
  std::cout << "[identity] " << kHubotClass << "  -> " << hubot_lib << std::endl;
  std::cout << "[identity] " << kVanillaClass << " -> " << vanilla_lib << std::endl;
  EXPECT_NE(hubot_lib, vanilla_lib)
    << "both class names resolved to the SAME library; the control is degenerate";
  EXPECT_NE(hubot_lib.find("hubot"), std::string::npos);
  EXPECT_EQ(vanilla_lib.find("hubot"), std::string::npos)
    << "the upstream class name resolved into this package's own library";
}

// B -- the real thing: live LayeredCostmap, pluginlib-loaded filter,
//      production updateMap().
TEST_F(PluginlibLiveCostmap, B_LiveLayeredCostmap_UpdateMap_DegradesInsteadOfAborting)
{
  ASSERT_TRUE(bringUp(kHubotClass));
  ASSERT_TRUE(filterReportsActive()) << "filter never became active";

  EXPECT_NO_THROW(driveUpdateMapTwice())
    << "LayeredCostmap::updateMap() holds zero try/catch, and "
       "Costmap2DROS::updateMap() wraps its call in none either. Anything "
       "escaping here leaves the costmap update thread with no handler above "
       "it -- that is the abort this package exists to prevent.";

  auto h = std::dynamic_pointer_cast<hubot::ZoneParameterFilter>(filter_);
  ASSERT_NE(h, nullptr);
  EXPECT_TRUE(h->enforcementDegraded())
    << "the latch is set only inside process(), which is reached only through "
       "CostmapFilter::updateCosts(). If this is false then updateMap() never "
       "arrived at this filter and the test above measured nothing.";

  EXPECT_DOUBLE_EQ(target_node_->get_parameter("readonly_speed").as_double(), 1.0)
    << "the read-only parameter must be untouched by the refused set";
}

// B2 — ⚑ THE HUMAN'S HALF, AND IT WAS NOT ASSERTED ANYWHERE UNTIL 2026-09-06.
//
// WHY (written before the act). Test B proves the ROBOT survives.
// That is one half of this package's claim and it is the half a machine cares
// about. The other half is Feature 2: upstream expressed "I could not enforce
// this zone" by killing the process; hubot replaces that with a sentence a
// PERSON can act on. If the process lives and says nothing a human can use,
// the package has removed an honest crash and put a silence in its place --
// which is worse for her, not better, because a silent robot that keeps
// driving is one she trusts.
//
// Measured 2026-09-06 across the whole suite before this test existed: exactly
// ONE assertion on `level` anywhere (degrade_at_production_caller_test.cpp:899)
// and it is on the UNDECLARED-MASK-STATE path, not on the rejected-parameter-set
// path that is this gate's subject. ZERO assertions on the message text, on any
// path. The sentence the README prints as the package's reason to exist was
// never checked by anything.
//
// This asserts it at the production caller: pluginlib-loaded class, live
// LayeredCostmap::updateMap(), the same refused set as test B.
TEST_F(PluginlibLiveCostmap, B2_LiveLayeredCostmap_PublishesTheHumanDecisionSurface)
{
  ASSERT_TRUE(bringUp(kHubotClass));
  ASSERT_TRUE(filterReportsActive()) << "filter never became active";

  EXPECT_NO_THROW(driveUpdateMapTwice());

  auto h = std::dynamic_pointer_cast<hubot::ZoneParameterFilter>(filter_);
  ASSERT_NE(h, nullptr);
  ASSERT_TRUE(pumpUntil([&] {return h->enforcementDegraded();}, 5s))
    << "the rejection was never exercised -- a vacuous run must not read as a "
       "pass, and every assertion below would be meaningless.";

  // The surface must actually arrive. `count()==0` with level 255 is how a
  // never-published topic looks, and it must not be mistaken for a verdict.
  ASSERT_TRUE(pumpUntil([&] {return decision_sub_->count() > 0;}, 5s))
    << "nothing was ever published on `zone_decision`. The integrator has no "
       "channel: enforcementDegraded() is public and NOTHING IN NAV2 CALLS IT.";

  ASSERT_TRUE(
    pumpUntil(
      [&] {
        return decision_sub_->lastLevel() ==
        diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      }, 5s))
    << "the zone is NOT being enforced and the surface does not say ERROR. "
       "level=" << static_cast<int>(decision_sub_->lastLevel())
    << " message=\"" << decision_sub_->lastMessage() << "\"";

  // ⚑ The MESSAGE, not only the level. A level is a number; the reason this
  // package exists is that a number is a robot's input. The sentence is the
  // artifact a person reads, so the sentence is what must be asserted.
  const std::string msg = decision_sub_->lastMessage();
  EXPECT_NE(msg.find("NOT being enforced"), std::string::npos)
    << "the message must say the zone is not being enforced. Got: \"" << msg << "\"";
  EXPECT_NE(msg.find("Decide as if the zone's limits are not applied"), std::string::npos)
    << "the message must tell the person what to DO about it -- that clause is "
       "the whole of Feature 2, and it is quoted verbatim on the package's "
       "README and in method_five_gated_build_2026_09_05.md. Got: \"" << msg << "\"";

  // And the machine-readable field an operator tool keys on.
  EXPECT_EQ(decision_sub_->value("enforced"), "NO")
    << "three-valued `enforced` must read NO when a target refused";
}

// C -- NEGATIVE CONTROL. Without this, A and B are not evidence.
//      Same loader, same costmap, same input, upstream's released filter.
//      It must throw out of updateMap().
TEST_F(PluginlibLiveCostmap, C_NegativeControl_UpstreamFilterEscapesUpdateMap)
{
  if (const std::string why = upstreamControlUnavailableReason(); !why.empty()) {
    GTEST_SKIP() << "no upstream control on this substrate -- " << why
                 << ". Without it, B is a property of this package that this file "
                    "cannot discriminate from a property of the harness; B still "
                    "runs, and this skip is the record that its control did not.";
  }

  ASSERT_TRUE(bringUp(kVanillaClass));

  // Assert WHY it threw. A control that throws for an unrelated reason -- a
  // missing node, a bad transform -- looks identical in a bare EXPECT_THROW and
  // would certify a harness that cannot see the real failure at all.
  // ISOLATION: the spin happens OUTSIDE the try, so a throw caught below can
  // only have come out of LayeredCostmap::updateMap() itself -- not from an
  // executor callback that merely ran nearby in time.
  std::string what;
  bool threw = false;
  bool first_call_threw = false;
  try {
    layers_->updateMap(1.5, 1.5, 0.0);
  } catch (const std::exception & ex) {
    first_call_threw = true; threw = true; what = ex.what();
  }
  if (!threw) {
    spinFor(400ms);                       // outside the try, deliberately
    try {
      layers_->updateMap(1.5, 1.5, 0.0);  // the ONLY statement under test
    } catch (const std::exception & ex) {
      threw = true; what = ex.what();
    }
  }
  std::cout << "[control] escaped_updateMap=" << threw
            << " on_first_call=" << first_call_threw << std::endl;
  std::cout << "[control] what=\"" << what << "\"" << std::endl;

  ASSERT_TRUE(threw)
    << "THE INSTRUMENT IS BLIND. Released nav2 is known to throw on this input. "
       "If it does not throw here, this harness cannot observe the failure it "
       "claims to rule out, and test B's pass is a property of the harness "
       "rather than of this package.";
  EXPECT_NE(what.find("set_parameters failed"), std::string::npos)
    << "the control threw, but not for the reason under test. what() was: "
    << what;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
