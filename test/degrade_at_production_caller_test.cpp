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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include <map>
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";

// Full parameter name under the filter's namespace.
std::string fp(const std::string & suffix)
{
  return std::string(kFilterName) + "." + suffix;
}

// Appends the declared-config overrides for one explicit
// {node, parameter, value} entry rooted at fp(prefix). Used both for
// `<state>.<override_name>` entries and `nominal_defaults.<name>` entries.
void addEntry(
  std::vector<rclcpp::Parameter> & cfg, const std::string & prefix,
  const std::string & target_node, const std::string & parameter,
  const rclcpp::ParameterValue & value)
{
  cfg.emplace_back(fp(prefix + ".node"), target_node);
  cfg.emplace_back(fp(prefix + ".parameter"), parameter);
  cfg.push_back(rclcpp::Parameter(fp(prefix + ".value"), value));
}
}  // namespace

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher(uint8_t type, const char * mask_topic, double base, double multiplier)
  : Node("costmap_filter_info_pub")
  {
    publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      kInfoTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = type;
    msg->filter_mask_topic = mask_topic;
    msg->base = static_cast<float>(base);
    msg->multiplier = static_cast<float>(multiplier);
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

class StateEventSubscriber : public rclcpp::Node
{
public:
  explicit StateEventSubscriber(const std::string & topic)
  : Node("zpf_state_sub"), last_state_(0), received_(false)
  {
    subscriber_ = create_subscription<std_msgs::msg::UInt8>(
      topic, rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::ConstSharedPtr msg) {
        last_state_ = msg->data;
        received_ = true;
      });
  }

  uint8_t lastState() const {return last_state_;}
  bool received() const {return received_;}

private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscriber_;
  uint8_t last_state_;
  bool received_;
};

static nav_msgs::msg::OccupancyGrid make_mask(uint32_t w, uint32_t h, int8_t fill_value)
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

class TargetNode : public rclcpp::Node
{
public:
  TargetNode()
  : rclcpp::Node("zpf_target_node")
  {
    declare_parameter("speed", 1.0);
    declare_parameter("inflation", 0.5);
    rcl_interfaces::msg::ParameterDescriptor ro_desc;
    ro_desc.read_only = true;
    declare_parameter("readonly_speed", 1.0, ro_desc);
  }

  double getSpeed() {return get_parameter("speed").as_double();}
  double getInflation() {return get_parameter("inflation").as_double();}
};

// A second explicit target: the declared config routes overrides by explicit
// `node` + `parameter` fields, so two overrides in one state can address two
// different nodes without any name disambiguation.
class SecondTargetNode : public rclcpp::Node
{
public:
  SecondTargetNode()
  : rclcpp::Node("zpf_second_target")
  {
    declare_parameter("speed", 1.0);
  }

  double getSpeed() {return get_parameter("speed").as_double();}
};

// ⚑ CPP 2026-09-05. `state_param_map_`, `nominal_defaults_` and
// `param_clients_` are protected, and whether resetFilter() actually empties
// them is a load-bearing question (N-1): CostmapFilter::reset() calls
// resetFilter() then initializeFilter(), so anything left behind is MERGED with
// the reload rather than replaced -- and nominal_defaults_ is filled with
// push_back. A subclass reads them without widening the public API.
class InspectableZpf : public hubot::ZoneParameterFilter
{
public:
  size_t declaredStates() const {return state_param_map_.size();}
  size_t nominalDefaultEntries() const
  {
    size_t n = 0;
    for (const auto & kv : nominal_defaults_) {n += kv.second.size();}
    return n;
  }
  size_t clients() const {return param_clients_.size();}
};

// Reads the `zone_decision` DiagnosticArray -- the surface an integrator
// actually has, since nothing in nav2 calls enforcementDegraded().
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
          last_message_ = st.message;
          last_level_ = st.level;
          for (const auto & kv : st.values) {
            last_[kv.key] = kv.value;
            seen_[kv.key].push_back(kv.value);
          }
        }
        count_++;
      });
  }

  std::string value(const std::string & k) const
  {
    auto it = last_.find(k);
    return it == last_.end() ? std::string("<absent>") : it->second;
  }
  // Every value this key has EVER carried, in order. `enforced` going
  // yes -> NO is invisible to a last-value read if the cycle is fast.
  std::vector<std::string> history(const std::string & k) const
  {
    auto it = seen_.find(k);
    return it == seen_.end() ? std::vector<std::string>{} : it->second;
  }
  bool everSaw(const std::string & k, const std::string & v) const
  {
    for (const auto & x : history(k)) {if (x == v) {return true;}}
    return false;
  }
  std::string lastMessage() const {return last_message_;}
  uint8_t lastLevel() const {return last_level_;}
  size_t count() const {return count_;}

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  std::map<std::string, std::string> last_;
  std::map<std::string, std::vector<std::string>> seen_;
  std::string last_message_;
  uint8_t last_level_{0};
  size_t count_{0};
};

class TestZpf : public ::testing::Test
{
protected:
  void SetUp() override
  {
    target_node_ = std::make_shared<TargetNode>();
    target_executor_.add_node(target_node_);
  }

  void TearDown() override
  {
    filter_.reset();
    info_pub_.reset();
    mask_pub_.reset();
    layers_.reset();
    if (state_event_sub_) {
      state_event_executor_.remove_node(state_event_sub_);
      state_event_sub_.reset();
    }
    if (decision_sub_) {
      decision_executor_.remove_node(decision_sub_);
      decision_sub_.reset();
    }
    if (node_) {
      node_executor_.remove_node(node_->get_node_base_interface());
    }
    node_.reset();
    if (second_target_node_) {
      target_executor_.remove_node(second_target_node_);
      second_target_node_.reset();
    }
    target_executor_.remove_node(target_node_);
    target_node_.reset();
  }

  // Builds a ZPF host node + filter + info/mask publishers. The filter's
  // state machine is injected as DECLARED configuration through
  // NodeOptions::parameter_overrides:
  //   <filter>.states                       : string array of state names
  //   <filter>.<state>.id                   : int64 mask value for the state
  //   <filter>.<state>.overrides            : string array of override names
  //   <filter>.<state>.<override>.node      : explicit target node
  //   <filter>.<state>.<override>.parameter : parameter on that node
  //   <filter>.<state>.<override>.value     : dynamically typed value
  //   <filter>.nominal_defaults             : string array; entries as above
  // Spins until the filter becomes active or 2s pass. Optional info_type
  // allows the wrong-type test to inject a non-ZPF info publisher.
  bool createFilter(
    const std::vector<rclcpp::Parameter> & filter_config,
    int8_t mask_fill_value,
    const std::string & state_event_topic = "",
    uint8_t info_type = hubot::kZoneParameterFilterType)
  {
    rclcpp::NodeOptions opts;
    std::vector<rclcpp::Parameter> all_overrides = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
    };
    if (!state_event_topic.empty()) {
      all_overrides.emplace_back(fp("state_event_topic"), state_event_topic);
    }
    for (const auto & p : filter_config) {
      all_overrides.push_back(p);
    }
    opts.parameter_overrides(all_overrides);

    node_ = std::make_shared<nav2::LifecycleNode>("zpf_test_host", opts);
    node_executor_.add_node(node_->get_node_base_interface());

    decision_sub_ = std::make_shared<DecisionSubscriber>();
    decision_executor_.add_node(decision_sub_);

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

    filter_ = std::make_shared<InspectableZpf>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>(info_type, kMaskTopic, 0.0, 1.0);
    auto mask = make_mask(4, 4, mask_fill_value);
    mask_pub_ = std::make_shared<MaskPublisher>(mask);
    pub_executor_.add_node(info_pub_);
    pub_executor_.add_node(mask_pub_);

    if (!state_event_topic.empty()) {
      state_event_sub_ = std::make_shared<StateEventSubscriber>(state_event_topic);
      state_event_executor_.add_node(state_event_sub_);
    }

    auto start = node_->now();
    while (!filter_->isActive()) {
      if (node_->now() - start > rclcpp::Duration(2s)) {
        return false;
      }
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      decision_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void rePublishMask(int8_t fill_value)
  {
    pub_executor_.remove_node(mask_pub_);
    mask_pub_.reset();
    auto mask = make_mask(4, 4, fill_value);
    mask_pub_ = std::make_shared<MaskPublisher>(mask);
    pub_executor_.add_node(mask_pub_);
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(150ms)) {
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      decision_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  void runProcess(double pose_x = 1.5, double pose_y = 1.5)
  {
    nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_x;
    pose.position.y = pose_y;
    pose.orientation.w = 1.0;
    filter_->process(costmap, 0, 0, 4, 4, pose);
  }

  template<typename Pred>
  bool waitForCond(Pred pred, std::chrono::milliseconds timeout = 1500ms)
  {
    auto start = node_->now();
    while (!pred()) {
      if (node_->now() - start > rclcpp::Duration(timeout)) {
        return false;
      }
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      decision_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  // ⚑ Like waitForCond, but TICKS THE FILTER each round.
  // checkPendingParameterUpdates() -- the thing that drains a future, clears a
  // fault, or resolves `pending` -- runs from the top of process().
  //
  // ⚑ CORRECTED 2026-09-06. The rest of this comment used to read "and nowhere
  // else ... which is inherent to a costmap filter (it owns no timer) and is
  // stated in AoU-5." That is no longer true and the claim about costmap
  // filters was never true: a liveness timer on the node now drives the same
  // function independently of the costmap (see `liveness_timer_`), so the
  // deadline fires and `pending` resolves with nothing ticking. These fixtures
  // still pump deliberately -- they leave `liveness_period` at its 1.0 s
  // default and assert on sub-second windows, so pumping is what makes them
  // deterministic rather than dependent on a heartbeat landing in time.
  // The liveness path has its own oracle in
  // test/costmap_silence_liveness_test.cpp.
  template<typename Pred>
  bool pumpUntil(Pred pred, std::chrono::milliseconds timeout = 3000ms)
  {
    auto start = node_->now();
    while (!pred()) {
      if (node_->now() - start > rclcpp::Duration(timeout)) {
        return false;
      }
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      decision_executor_.spin_some();
      runProcess();
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  void spinFor(std::chrono::milliseconds duration)
  {
    auto start = node_->now();
    while (node_->now() - start < rclcpp::Duration(duration)) {
      pub_executor_.spin_some();
      node_executor_.spin_some();
      target_executor_.spin_some();
      state_event_executor_.spin_some();
      decision_executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<TargetNode> target_node_;
  std::shared_ptr<SecondTargetNode> second_target_node_;
  std::shared_ptr<StateEventSubscriber> state_event_sub_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  nav2::TransformBuffer::SharedPtr tf_buffer_;
  std::shared_ptr<InspectableZpf> filter_;
  std::shared_ptr<DecisionSubscriber> decision_sub_;
  rclcpp::executors::SingleThreadedExecutor decision_executor_;
  std::shared_ptr<InfoPublisher> info_pub_;
  std::shared_ptr<MaskPublisher> mask_pub_;
  rclcpp::executors::SingleThreadedExecutor node_executor_;
  rclcpp::executors::SingleThreadedExecutor target_executor_;
  rclcpp::executors::SingleThreadedExecutor pub_executor_;
  rclcpp::executors::SingleThreadedExecutor state_event_executor_;
};

// ⚑ CPP 2026-09-05. THE HARNESS ABOVE THIS LINE IS UPSTREAM'S OWN, VERBATIM,
// from `git show upstream/lyrical:nav2_costmap_2d/test/unit/zone_parameter_filter_test.cpp`,
// with exactly two substitutions: the include, and the class under test.
//
// WHY (written before the act):
//   * upstream's suite drives `filter_->process(...)` directly and NEVER calls
//     `updateCosts()` -- 0 occurrences in its 770 lines, and 0 in ALL SIX
//     costmap-filter test files at `lyrical`. `CostmapFilter::updateCosts()`
//     (costmap_filter.cpp:132) is BARE and is the caller production uses. BDE
//     reproduced the abort through it: the process DIES, message-matched, with
//     a failing negative control.
//   * hubot exists to make that same input degrade instead of abort. A claim
//     like that is worth nothing asserted. So this file runs UPSTREAM'S OWN
//     FAILING INPUT through THE PRODUCTION CALLER and demands the OPPOSITE
//     outcome -- and the death-test control is INVERTED: it must NOT die.
//   * `scripts/ct-production-caller-test-gate.py` HALTED hubot before this file
//     existed. It should now PASS.

namespace {

std::vector<rclcpp::Parameter> rejectingConfig(
  const std::function<std::string(const std::string &)> & fp,
  const std::function<void(std::vector<rclcpp::Parameter> &, const std::string &,
    const std::string &, const std::string &, const rclcpp::ParameterValue &)> & addEntry)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"danger_zone"}),
    rclcpp::Parameter(fp("danger_zone.id"), 1),
    rclcpp::Parameter(fp("danger_zone.setpoints"), std::vector<std::string>{"ro_speed"}),
  };
  addEntry(cfg, "danger_zone.ro_speed", "zpf_target_node", "readonly_speed",
    rclcpp::ParameterValue(0.5));
  return cfg;
}

}  // namespace

// A — the inversion. Upstream's `ParamSetFailureAlwaysThrows` asserts
//     EXPECT_THROW on this exact input. hubot must NOT throw.
TEST_F(TestZpf, HUBOT_A_TheSameInputThatThrowsUpstream_DoesNotThrowHere)
{
  auto cfg = rejectingConfig([this](const std::string & s) {return fp(s);},
      [this](std::vector<rclcpp::Parameter> & c, const std::string & a,
      const std::string & b, const std::string & d, const rclcpp::ParameterValue & v)
      {addEntry(c, a, b, d, v);});
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  auto drive_and_drain = [this]() {runProcess(); spinFor(300ms); runProcess();};
  EXPECT_NO_THROW(drive_and_drain())
    << "upstream throws std::runtime_error here; degrading instead is the whole "
       "reason this package exists";
  EXPECT_TRUE(filter_->enforcementDegraded())
    << "silence is not degrading. The failure must be LATCHED and readable.";
}

// B — THE PRODUCTION CALLER. This is what CT's S26 gate demands and what
//     upstream's 770 lines never do.
TEST_F(TestZpf, HUBOT_B_ProductionCaller_updateCosts_SurvivesTheRejection)
{
  auto cfg = rejectingConfig([this](const std::string & s) {return fp(s);},
      [this](std::vector<rclcpp::Parameter> & c, const std::string & a,
      const std::string & b, const std::string & d, const rclcpp::ParameterValue & v)
      {addEntry(c, a, b, d, v);});
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
  auto drive = [&]() {
      filter_->updateBounds(1.5, 1.5, 0.0, nullptr, nullptr, nullptr, nullptr);
      filter_->updateCosts(costmap, 0, 0, 4, 4);
      spinFor(300ms);
      filter_->updateBounds(1.5, 1.5, 0.0, nullptr, nullptr, nullptr, nullptr);
      filter_->updateCosts(costmap, 0, 0, 4, 4);
    };
  EXPECT_NO_THROW(drive())
    << "CostmapFilter::updateCosts() is bare; anything escaping it reaches "
       "LayeredCostmap, which has no handler anywhere";
  EXPECT_TRUE(filter_->enforcementDegraded());
  EXPECT_DOUBLE_EQ(target_node_->get_parameter("readonly_speed").as_double(), 1.0)
    << "the read-only parameter must be untouched by the failed set";
}

// C — ⚑ REMOVED, AND THE REASON IS THE FINDING.
//
// This slot held an inverted death test: EXPECT_EXIT(..., ExitedWithCode(0)) around
// the same threaded updateCosts() block BDE used to prove the abort upstream. It
// DEADLOCKED -- 3 minutes of silence, container alive, exactly what gtest warns:
// "Death tests use fork(), which is unsafe particularly in a threaded context.
//  For this test, Google Test detected 18 threads ... especially if this is the
//  last message you see before your test times out."
//
// The asymmetry is real and worth carrying: fork() clones ONLY the calling thread,
// so any mutex another thread held at fork time is held forever in the child.
// BDE's test survives that because its child ABORTS almost immediately. A test
// asserting SURVIVAL needs the child to keep working -- which is precisely what
// fork-in-a-threaded-ROS-process cannot do.
//
//   A death test can reliably observe a CRASH. It cannot reliably observe SURVIVAL.
//
// Test B above is the correct instrument for survival: it calls the real
// updateCosts() in-process, with no fork, and asserts EXPECT_NO_THROW. Removing
// this rather than leaving it disabled, because a hanging test in the suite is a
// worse loom than no test -- it teaches the next runner to pass -j and look away.

// D — ⚑ RE-AUTHORED 2026-09-05, AND THE OLD ASSERTION WAS THE DEFECT.
//
// This slot held HUBOT_D_AReloadClearsTheLatch_TheContractTheHeaderStates,
// which required resetFilter() to CLEAR enforcement_degraded_, on the header's
// stated ground that "the configuration it referred to is gone". It passed. It
// was testing AAA's N-1 finding into permanence.
//
// The ground was false twice over, both measured 2026-09-05:
//   * The configuration was not going anywhere. state_param_map_,
//     nominal_defaults_ and param_clients_ had NO `.clear()` anywhere in the
//     translation unit -- zero occurrences each -- while
//     CostmapFilter::reset() (costmap_filter.cpp:103) is
//     `resetFilter(); initializeFilter(...);`, so loadStateConfig() re-ran over
//     those same containers and nominal_defaults_ is filled with push_back.
//   * The flag is a claim about a TARGET NODE'S live parameter value. Nothing
//     in a costmap-filter reload reaches the target, so nothing about the
//     reload refutes it. And the path is ORDINARY: ClearEntireCostmap sits in
//     SEVEN of nav2's default behaviour trees and arrives here through
//     Costmap2DROS::resetLayers() (costmap_2d_ros.cpp:719).
//
// A test that passes because the code and the comment agree with each other,
// while both are wrong about the world, is the most expensive kind. The three
// tests below replace it and assert the corrected contract.

// D1 — the fault SURVIVES the recovery that used to erase it.
TEST_F(TestZpf, HUBOT_D1_AReloadDoesNotEraseAFaultThatStillStands)
{
  auto cfg = rejectingConfig([this](const std::string & s) {return fp(s);},
      [this](std::vector<rclcpp::Parameter> & c, const std::string & a,
      const std::string & b, const std::string & d, const rclcpp::ParameterValue & v)
      {addEntry(c, a, b, d, v);});
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  spinFor(300ms);
  runProcess();
  ASSERT_TRUE(filter_->enforcementDegraded()) << "precondition: the latch must be set";

  filter_->resetFilter();

  EXPECT_TRUE(filter_->enforcementDegraded())
    << "ClearEntireCostmap is in seven of nav2's default behaviour trees and "
       "reaches resetFilter(). If a routine recovery clears this flag, the one "
       "signal an integrator has disappears while the target is still not "
       "carrying the zone's values -- and the next thing they read is "
       "`enforced: yes`.";
}

// D2 — the reload DOES drop the configuration, which is what the header always
//      claimed was happening and what nothing in the file actually did.
TEST_F(TestZpf, HUBOT_D2_AReloadActuallyDropsTheConfiguration)
{
  auto cfg = rejectingConfig([this](const std::string & s) {return fp(s);},
      [this](std::vector<rclcpp::Parameter> & c, const std::string & a,
      const std::string & b, const std::string & d, const rclcpp::ParameterValue & v)
      {addEntry(c, a, b, d, v);});
  cfg.emplace_back(fp("nominal_defaults"), std::vector<std::string>{"base_speed"});
  addEntry(cfg, "nominal_defaults.base_speed", "zpf_target_node", "readonly_speed",
    rclcpp::ParameterValue(1.0));
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  const size_t states0 = filter_->declaredStates();
  const size_t nominals0 = filter_->nominalDefaultEntries();
  const size_t clients0 = filter_->clients();
  ASSERT_GT(states0, 0u);
  ASSERT_GT(nominals0, 0u);

  // Exactly what CostmapFilter::reset() does, in its order.
  filter_->resetFilter();
  EXPECT_EQ(filter_->declaredStates(), 0u) << "resetFilter() must drop the state map";
  EXPECT_EQ(filter_->nominalDefaultEntries(), 0u) << "and the nominal defaults";
  EXPECT_EQ(filter_->clients(), 0u) << "and the parameter clients";

  filter_->initializeFilter(kInfoTopic);
  EXPECT_EQ(filter_->declaredStates(), states0);
  EXPECT_EQ(filter_->clients(), clients0);
  EXPECT_EQ(filter_->nominalDefaultEntries(), nominals0)
    << "nominal_defaults_ is filled with push_back, so if resetFilter() does "
       "not clear it, EVERY ClearEntireCostmap doubles this number without "
       "bound. Seven default behaviour trees issue that recovery.";
}

// D3 — and the ONLY thing that retracts a fault is the thing that refutes it.
TEST_F(TestZpf, HUBOT_D3_AFaultIsRetractedOnlyByASetThatComesBackSuccessful)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"bad_zone", "good_zone"}),
    rclcpp::Parameter(fp("bad_zone.id"), 1),
    rclcpp::Parameter(fp("bad_zone.setpoints"), std::vector<std::string>{"ro"}),
    rclcpp::Parameter(fp("good_zone.id"), 2),
    rclcpp::Parameter(fp("good_zone.setpoints"), std::vector<std::string>{"ok"}),
  };
  // Rejected: read-only. Accepted: an ordinary parameter on the same node, so
  // the retraction is unambiguously about THAT target.
  addEntry(cfg, "bad_zone.ro", "zpf_target_node", "readonly_speed",
    rclcpp::ParameterValue(0.5));
  addEntry(cfg, "good_zone.ok", "zpf_target_node", "speed",
    rclcpp::ParameterValue(0.3));
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  spinFor(400ms);
  runProcess();
  ASSERT_TRUE(filter_->enforcementDegraded()) << "precondition: rejection must latch";

  rePublishMask(2);
  ASSERT_TRUE(pumpUntil([this] {return !filter_->enforcementDegraded();}, 4000ms))
    << "a set to the SAME target came back successful, which is the only "
       "evidence that can refute 'this target is not carrying the values'";
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 0.3);
}

// ===========================================================================
// ⚑ FSE 2026-09-05 — THE TWO CASES NOBODY ASKED FOR.
//
// Every test above this line asks "can we prove it DEGRADES?".  Neither asks
// whether degrading is the SAFE answer, and neither covers the two conditions
// most likely to occur on a real robot.  Both tests below FAIL against the
// committed source.  That is deliberate: they state the invariant the design
// needs, and the code does not hold it yet.  A suite that cannot fail on the
// real defect has measured nothing.
//
// HONEST BOUND: written on a host with NO ROS toolchain (`/opt/ros` absent,
// no colcon, no ros2 — measured 2026-09-05).  These are NOT compiled and NOT
// run.  BDE/FBR own the build.  Do not read them as passing, and do not read
// them as compiling.
// ===========================================================================

// E — ⚑ WITHDRAWN BEFORE IT WAS EVER RUN, AND THE REASON IS THE DISCIPLINE.
//
// This slot held FSE_E_AnUndeclaredMaskValue_MustDegrade_NotAbort: drive a mask
// cell of 7 against a config that declares only state 1, and demand no abort.
// It was written, then withdrawn on the §11 re-entrancy test — "does the
// primitive already exist under another name? If yes, duplication-class: rename
// the question, do NOT invent the loom."
//
// It does exist: test/zpf_survival_probe.cpp mode `unknown-state`, driven by
// test/survival_harness_test.cpp:213, authored the same day by another seat.
// Theirs is the BETTER instrument and this one would have been the worse:
// applyState() throwing on an undeclared mask value is an ABORT-class failure,
// and an in-process EXPECT_NO_THROW cannot observe a process that dies by
// abort, exit, terminate-on-another-thread, or deadlock. The fork+execv probe
// can. Two tests of the same condition where the weaker one looks green is how
// a suite starts lying.
//
// ⚑ The finding survives the withdrawal and is NOT closed by it: the throw at
// src/zone_parameter_filter.cpp:407 is still live, still on the costmap thread,
// still reached from CostmapFilter::updateCosts(), and it is reached by MASK
// DATA rather than by configuration. INV-A in test/safety_invariants_static.sh
// asserts it with no toolchain at all.

// F — A TARGET THAT NEVER ANSWERS IS REPORTED AS SUCCESS, FOREVER.
//
// THE MOST IMPORTANT TEST IN THIS FILE.
//
// issueAsyncSetParameters() pushes a future; checkPendingParameterUpdates()
// polls with wait_for(0s).  A future that never becomes ready is never
// examined, so there is no failure, no latch, and enforcementDegraded() stays
// FALSE — while publishDecision() reports `enforced: yes` and "Zone N is in
// force."  kMaxPendingSets is declared to bound exactly this and is referenced
// zero times.
//
// This is the single most likely failure on a real robot: the target node
// crashed, was never brought up, is still in its configuring transition, or the
// namespace in the YAML has a typo.  Every one of those produces a
// success-shaped report while the zone's limits are not applied and the robot
// keeps driving.  A rejection is the EASY case — someone answered.  Silence is
// the hard one, and it is uncovered.
//
// ⚑ WHY THIS ONE IS AN IN-PROCESS GTEST AND NOT A SURVIVAL-PROBE MODE.
// The probe exists to observe a process that DIES. This failure is the exact
// opposite: nothing dies, nothing throws, nothing logs — the filter returns
// normally and reports success. Silence needs an in-process assertion; only
// abort needs the fork+execv boundary. If the probe's owner would rather carry
// it as a third mode (`silent-target`), it belongs there and this can go.
//
// Expected mechanism (BDE to confirm): AsyncParametersClient::set_parameters
// against a node that does not exist sends a request to no service and returns
// a future that never completes. It does not error.
TEST_F(TestZpf, FSE_F_ATargetThatNeverAnswers_MustNotReportEnforcement)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"danger_zone"}),
    rclcpp::Parameter(fp("danger_zone.id"), 1),
    rclcpp::Parameter(fp("danger_zone.setpoints"), std::vector<std::string>{"slow"}),
  };
  // A node nobody ever creates. A client IS still built for it at config-load.
  addEntry(cfg, "danger_zone.slow", "zpf_node_that_does_not_exist", "speed",
    rclcpp::ParameterValue(0.2));
  // ⚑ CPP 2026-09-05: the deadline this test depends on, named explicitly.
  // The shipped default is 5.0s and this test spins for 2s, so without this
  // line the test was asserting a promise the design does not make -- it would
  // have stayed red against a correct implementation. The mechanism under test
  // is the deadline, not its default.
  cfg.emplace_back(fp("set_parameters_timeout"), 0.5);

  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  spinFor(2000ms);
  runProcess();

  EXPECT_TRUE(filter_->enforcementDegraded())
    << "No answer arrived within 2s, so nothing confirms the zone's limits were "
       "applied — and the filter reports enforcement anyway. An unanswered "
       "request is not a successful one. The zone must time out and degrade "
       "LOUDLY, exactly as a rejection does; otherwise the one failure a "
       "person cannot see is the one most likely to happen.";
}

// ===========================================================================
// ⚑ CPP 2026-09-05 — AAA's PUSHBACKS, TURNED INTO INSTRUMENTS.
// AAA ruled SPLIT on the unknown-state patch: the throw goes, and three things
// were wrong with the patch as written. P-1 and N-3 are asserted below. Each
// was a claim; each is now a test that goes red on the patch AAA refused.
// ===========================================================================

// P1 — ⚑ AAA's HIGHEST-VALUE PUSHBACK, AND IT IS CORRECT.
//
// Removing the throw is not the fix; it is half of it. After applyState()
// degrades, control still reached `current_state_ = new_state`, so the filter
// recorded a state it had NOT applied. The next transition's N-only reset then
// looked up that undeclared id in state_param_map_, missed, and reset NOTHING
// -- carrying the PREVIOUS zone's speed limit into a zone that never declared
// it, invisibly, until some later exit-to-nominal.
//
// The negative control for this test is NOT the shipped tree (which aborts
// here and cannot be observed in-process). It is the degrade patch as AAA
// received it: throw -> log-and-return, with the assignment left standing.
// Against that build, `speed` below stays at 0.2.
TEST_F(TestZpf, CPP_P1_AnUndeclaredStateMustNotBeRecordedAsTheCurrentState)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone", "wide_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"sp"}),
    rclcpp::Parameter(fp("wide_zone.id"), 2),
    rclcpp::Parameter(fp("wide_zone.setpoints"), std::vector<std::string>{"inf"}),
    rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"n_speed", "n_inf"}),
  };
  addEntry(cfg, "slow_zone.sp", "zpf_target_node", "speed", rclcpp::ParameterValue(0.2));
  addEntry(cfg, "wide_zone.inf", "zpf_target_node", "inflation", rclcpp::ParameterValue(0.9));
  addEntry(cfg, "nominal_defaults.n_speed", "zpf_target_node", "speed",
    rclcpp::ParameterValue(1.0));
  addEntry(cfg, "nominal_defaults.n_inf", "zpf_target_node", "inflation",
    rclcpp::ParameterValue(0.5));
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  // Zone 1: speed limited.
  runProcess();
  ASSERT_TRUE(waitForCond([this] {return target_node_->getSpeed() == 0.2;}, 3000ms))
    << "precondition: zone 1 must actually take effect";

  // Zone 7: declared nowhere. Degrade, change nothing, record nothing.
  rePublishMask(7);
  runProcess();
  spinFor(300ms);
  runProcess();
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 0.2)
    << "an undeclared zone must not silently alter anything";

  // Zone 2: sets inflation only. The N-only reset from zone 1 must restore
  // speed -- which requires that the filter still knows it is in zone 1.
  rePublishMask(2);
  runProcess();
  ASSERT_TRUE(waitForCond([this] {return target_node_->getInflation() == 0.9;}, 3000ms))
    << "zone 2 must take effect";
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 1.0)
    << "⚑ P-1. Zone 2 does not declare `speed`, so entering it must reset "
       "speed to its nominal default. That reset is computed from the state "
       "the filter believes it is leaving. If the undeclared value 7 was "
       "recorded as current, the lookup misses and zone 1's 0.2 m/s limit "
       "rides into zone 2 -- a limit no configuration ever asked for, on a "
       "robot whose operator has no way to see it.";
}

// N3 — the enter path published `enforced: yes` with zero confirmations.
//
// The leave path already refuses to do this, and says so in the header: an
// ISSUED restore is not a CONFIRMED one. The enter path did it anyway on the
// same cycle the async sets went out. Read through the DiagnosticArray, which
// is the surface an integrator actually has.
TEST_F(TestZpf, CPP_N3_TheEnterPathMustNotPublishEnforcedYesBeforeAnyConfirmation)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"sp"}),
  };
  addEntry(cfg, "slow_zone.sp", "zpf_target_node", "speed", rclcpp::ParameterValue(0.2));
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  // ONE cycle. The sets have been issued and nothing can have answered.
  // Wait for the TRANSITION publish specifically (zone_state 1), not merely
  // for any message: `waitForCond` returns on the first one delivered, and
  // racing the second is how this test read a half-empty history and blamed
  // the code.
  runProcess();
  ASSERT_TRUE(waitForCond([this] {return decision_sub_->value("zone_state") == "1";}, 2000ms))
    << "the decision surface must publish on a transition";

  EXPECT_FALSE(decision_sub_->everSaw("enforced", "yes"))
    << "⚑ N-3. On the cycle the sets are ISSUED, nothing has confirmed them. "
       "Publishing `enforced: yes` here is the success-shaped value this "
       "package's own header says the feature exists to abolish -- the code "
       "applied that principle on the leave path and violated it on the enter "
       "path. `pending` is the honest third value.";
  EXPECT_TRUE(decision_sub_->everSaw("enforced", "pending"))
    << "requested-and-unconfirmed must be SAYABLE, not rounded to yes or NO";

  // And it must resolve to yes once the target answers -- `pending` that never
  // clears would be its own lie.
  ASSERT_TRUE(
    pumpUntil([this] {return decision_sub_->value("enforced") == "yes";}, 4000ms))
    << "once the target confirms, the honest value is yes";
  EXPECT_DOUBLE_EQ(target_node_->getSpeed(), 0.2);
}

// N3b — the undeclared zone must be readable on the same surface, with BOTH
//       numbers, because their disagreement is the whole fault.
TEST_F(TestZpf, CPP_N3b_TheDecisionSurfaceCarriesBothTheMaskStateAndTheAppliedState)
{
  std::vector<rclcpp::Parameter> cfg = {
    rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow_zone"}),
    rclcpp::Parameter(fp("slow_zone.id"), 1),
    rclcpp::Parameter(fp("slow_zone.setpoints"), std::vector<std::string>{"sp"}),
  };
  addEntry(cfg, "slow_zone.sp", "zpf_target_node", "speed", rclcpp::ParameterValue(0.2));
  ASSERT_TRUE(createFilter(cfg, 1)) << "Filter did not become active";

  runProcess();
  ASSERT_TRUE(waitForCond([this] {return target_node_->getSpeed() == 0.2;}, 3000ms));

  rePublishMask(7);
  runProcess();
  ASSERT_TRUE(waitForCond([this] {return filter_->maskStateUndeclared();}, 2000ms));
  spinFor(300ms);

  EXPECT_EQ(decision_sub_->value("mask_state"), "7")
    << "where the mask says the robot is";
  EXPECT_EQ(decision_sub_->value("zone_state"), "1")
    << "and what is actually in force. One number cannot carry both, and it is "
       "their disagreement that tells a person something is wrong.";
  EXPECT_EQ(decision_sub_->value("enforced"), "NO");
  EXPECT_EQ(decision_sub_->lastLevel(), diagnostic_msgs::msg::DiagnosticStatus::ERROR);
}

// upstream's own main(), verbatim — the fixture needs rclcpp::init before it
// can create a guard condition. Dropped by the first extraction; the failure it
// produced was environmental ("context argument is null"), not behavioural.
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
