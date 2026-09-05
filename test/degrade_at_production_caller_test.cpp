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
    uint8_t info_type = nav2_costmap_2d::ZONE_PARAMETER_FILTER)
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

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);  // One-thread broadcasting-listening model

    filter_ = std::make_shared<hubot::ZoneParameterFilter>();
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
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<TargetNode> target_node_;
  std::shared_ptr<SecondTargetNode> second_target_node_;
  std::shared_ptr<StateEventSubscriber> state_event_sub_;
  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  nav2::TransformBuffer::SharedPtr tf_buffer_;
  std::shared_ptr<hubot::ZoneParameterFilter> filter_;
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
// WHY (written before the act, OPS-070(B)):
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

// D — the OWED test named in test/test_degrade_not_abort.cpp, and the one the
//     cross-SWOT measured as FAILING against the shipped code. It now passes
//     because resetFilter() was fixed to keep the contract its header states.
TEST_F(TestZpf, HUBOT_D_AReloadClearsTheLatch_TheContractTheHeaderStates)
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
  EXPECT_FALSE(filter_->enforcementDegraded())
    << "the header states 'a reload clears it because the configuration it "
       "referred to is gone'. Before 2026-09-05 the code did not do this, and "
       "this test would have FAILED against the shipped contract.";
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
