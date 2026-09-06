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

// =============================================================================
// THE MAINTAINER-FINDING HARNESS.
//
// nav2's maintainer reviewed the upstream sibling of this filter three times and
// found something every time. His own words, on the third round:
//
//   "every time finding several (which maybe you can catch yourself? Not sure
//    why I'm catching them but you're not)."
//
// He is right, and the reason is mechanical rather than moral. It was measured
// and written down before this file existed:
//
//   "Every one is about what happens ACROSS a sequence -- lifecycle transitions,
//    or message arrival order. Not one is 'this function computes the wrong
//    value.' Our 486 passing tests test FUNCTIONS. His findings live in
//    SEQUENCES. A value-oracle cannot fail on a sequence defect."
//
// So this file is not a value oracle. Every case here drives a SEQUENCE and
// asserts INSIDE a transient, never on the settled state either side of it.
//
// -----------------------------------------------------------------------------
// THE SHAPE THIS FILE MUST NOT HAVE, named so it cannot come back by habit.
//
// The upstream suite could not fail on any of his findings because its setup
// helper waited the defect out. Here is that helper, verbatim:
//
//     bool reloadFilter()
//     {
//       filter_->resetFilter();
//       filter_->initializeFilter(kInfoTopic);
//       auto start = node_->now();
//       while (!filter_->isActive()) {        // <-- spins until the mask is back
//         if (node_->now() - start > rclcpp::Duration(2s)) {return false;}
//         spinOnce();
//         std::this_thread::sleep_for(10ms);
//       }
//       return true;
//     }
//
// `isActive()` is `filter_mask_ != nullptr`. So the helper blocks until the
// latched mask has been redelivered on the new subscription -- which is the
// precise window every one of his blocking findings lives in. A test built on
// that helper cannot observe the defect it was written for, and will pass.
//
// The fixture below therefore has NO wait-for-active after a reset. It calls
// `reset()` -- the production entry point, what ClearEntireCostmap reaches
// through Costmap2DROS::resetLayers() -- and then asserts immediately.
//
// -----------------------------------------------------------------------------
// SCOPE. His findings are against the UPSTREAM filter's proposed changes, not
// against this package. This file does not port them; it asks, per finding,
// whether the same defect can occur HERE, and only writes a test where the
// measurement says it can. `doc/SPEC_COVERAGE.md` carries the per-finding
// verdict and the reason for each one marked cannot-occur -- because "does not
// apply here" is the sentence under which a defect survives a sweep, and it has
// to be paid for.
// =============================================================================

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/tf2_factories.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;  // NOLINT(build/namespaces)

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";
constexpr int8_t kZoneCell = 3;        // the mask value that selects `slow`
constexpr double kZoneSpeed = 0.2;     // what the zone imposes
constexpr double kNominalSpeed = 1.0;  // what nominal defaults restore

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
}  // namespace

class InfoPublisher : public rclcpp::Node
{
public:
  InfoPublisher()
  : Node("mf_info_pub")
  {
    publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>(
      kInfoTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    auto msg = std::make_unique<nav2_msgs::msg::CostmapFilterInfo>();
    msg->type = nav2_costmap_2d::ZONE_PARAMETER_FILTER;
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
  : Node("mf_mask_pub")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }
  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

// The node whose parameter the zone actually changes. Its LIVE VALUE is the
// second of the two facts: what the filter CLAIMS is in force, and what IS.
class TargetNode : public rclcpp::Node
{
public:
  TargetNode()
  : rclcpp::Node("mf_target_node")
  {
    declare_parameter("speed", kNominalSpeed);
  }
  double liveSpeed() {return get_parameter("speed").as_double();}
};

// The observer, on its own node. It records every message with its arrival
// time so a test can reason about the SEQUENCE, not only the last value.
class DecisionRecorder : public rclcpp::Node
{
public:
  struct Sample
  {
    std::map<std::string, std::string> values;
    uint8_t level;
    std::string message;
    double arrived_at_s;
  };

  DecisionRecorder()
  : Node("mf_decision_recorder"), t0_(std::chrono::steady_clock::now())
  {
    subscriber_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "zone_decision", rclcpp::QoS(400),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        const double t = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - t0_).count();
        std::lock_guard<std::mutex> lk(m_);
        for (const auto & st : msg->status) {
          Sample s;
          s.level = st.level;
          s.message = st.message;
          s.arrived_at_s = t;
          for (const auto & kv : st.values) {
            s.values[kv.key] = kv.value;
          }
          samples_.push_back(std::move(s));
        }
      });
  }

  std::vector<Sample> since(size_t mark) const
  {
    std::lock_guard<std::mutex> lk(m_);
    if (mark >= samples_.size()) {return {};}
    return std::vector<Sample>(samples_.begin() + mark, samples_.end());
  }
  size_t mark() const
  {
    std::lock_guard<std::mutex> lk(m_);
    return samples_.size();
  }

private:
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr subscriber_;
  mutable std::mutex m_;
  std::vector<Sample> samples_;
  std::chrono::steady_clock::time_point t0_;
};

namespace
{
std::string get(const DecisionRecorder::Sample & s, const std::string & k)
{
  auto it = s.values.find(k);
  return it == s.values.end() ? std::string("<absent>") : it->second;
}

std::string render(const DecisionRecorder::Sample & s)
{
  return "t=" + std::to_string(s.arrived_at_s) +
         " level=" + std::to_string(static_cast<int>(s.level)) +
         " enforced=" + get(s, "enforced") +
         " watching=" + get(s, "watching") +
         " configured=" + get(s, "configured") +
         " zone_state=" + get(s, "zone_state") +
         " mask_state=" + get(s, "mask_state") +
         " seq=" + get(s, "report_seq") +
         " msg=\"" + s.message + "\"";
}
}  // namespace

class MaintainerFindings : public ::testing::Test
{
protected:
  void TearDown() override
  {
    stopSpinning();
    filter_.reset();
    info_pub_.reset();
    mask_pub_.reset();
    layers_.reset();
    recorder_.reset();
    node_.reset();
    target_node_.reset();
  }

  // `target_exists=false` routes the zone's set at a node nobody is running, so
  // the future never becomes ready: a set that is genuinely IN FLIGHT and will
  // never be answered. That is the state the clear has to be honest about.
  bool build(
    int8_t mask_fill, bool target_exists = true,
    double liveness_period = 0.1, double silence_timeout = 2.0,
    double set_timeout = 30.0)
  {
    std::vector<rclcpp::Parameter> cfg = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
      rclcpp::Parameter(fp("liveness_period"), liveness_period),
      rclcpp::Parameter(fp("costmap_silence_timeout"), silence_timeout),
      rclcpp::Parameter(fp("set_parameters_timeout"), set_timeout),
      rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow"}),
      rclcpp::Parameter(fp("slow.id"), static_cast<int64_t>(kZoneCell)),
      rclcpp::Parameter(fp("slow.setpoints"), std::vector<std::string>{"cap"}),
      rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"cap"}),
    };
    const std::string target =
      target_exists ? "mf_target_node" : "a_node_that_is_not_running";
    addEntry(cfg, "slow.cap", target, "speed", rclcpp::ParameterValue(kZoneSpeed));
    addEntry(
      cfg, "nominal_defaults.cap", target, "speed",
      rclcpp::ParameterValue(kNominalSpeed));

    rclcpp::NodeOptions opts;
    opts.parameter_overrides(cfg);

    if (target_exists) {target_node_ = std::make_shared<TargetNode>();}
    node_ = std::make_shared<nav2::LifecycleNode>("mf_host", opts);
    recorder_ = std::make_shared<DecisionRecorder>();

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);

    filter_ = std::make_shared<hubot::ZoneParameterFilter>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>();
    mask_pub_ = std::make_shared<MaskPublisher>(make_mask(4, 4, mask_fill));

    startSpinning();
    return true;
  }

  bool waitActive(std::chrono::milliseconds budget = 5000ms)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    while (!filter_->isActive()) {
      if (std::chrono::steady_clock::now() > deadline) {return false;}
      std::this_thread::sleep_for(5ms);
    }
    return true;
  }

  // Production topology: every node spins on a thread that is NOT the test
  // thread, exactly as nav2::NodeThread does for the costmap node
  // (controller_server.cpp:72). The costmap update loop is driven from the test
  // thread, so stopping one does not stop the other.
  void startSpinning()
  {
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_->get_node_base_interface());
    exec_->add_node(recorder_);
    exec_->add_node(info_pub_);
    exec_->add_node(mask_pub_);
    if (target_node_) {exec_->add_node(target_node_);}
    spin_thread_ = std::thread([this]() {exec_->spin();});
  }

  void stopSpinning()
  {
    if (exec_) {exec_->cancel();}
    if (spin_thread_.joinable()) {spin_thread_.join();}
    exec_.reset();
  }

  // The production caller: updateBounds() then updateCosts(). Not process().
  void tickCostmap(double x = 1.5, double y = 1.5)
  {
    nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
    double mnx = 0, mny = 0, mxx = 0, mxy = 0;
    filter_->updateBounds(x, y, 0.0, &mnx, &mny, &mxx, &mxy);
    filter_->updateCosts(costmap, 0, 0, 4, 4);
  }

  // THE ROUTINE RECOVERY, through the production entry point.
  // `Layer::reset()` is what Costmap2DROS::resetLayers() calls
  // (costmap_2d_ros.cpp:719), which is what the `clear_entirely_<costmap>`
  // service reaches, which appears in seven of nav2's default behaviour trees.
  // CostmapFilter::reset() is `resetFilter(); initializeFilter(...);
  // setCurrent(false);` (costmap_filter.cpp:103-108).
  //
  // AND IT RETURNS IMMEDIATELY. No wait-for-active. That is the whole point of
  // this file: the caller must be able to assert inside the window.
  void routineClearNoWait() {filter_->reset();}

  void settle(std::chrono::milliseconds d) {std::this_thread::sleep_for(d);}

  std::shared_ptr<nav2::LifecycleNode> node_;
  std::shared_ptr<TargetNode> target_node_;
  std::shared_ptr<DecisionRecorder> recorder_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<hubot::ZoneParameterFilter> filter_;
  std::shared_ptr<InfoPublisher> info_pub_;
  std::shared_ptr<MaskPublisher> mask_pub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;
};

// ===========================================================================
// MF-1 -- NOT-YET-DRIVEN MUST NOT READ AS ENFORCED.
//
// His first finding is that a routine clear un-enforces the zone while the
// surface still announces it. The upstream MECHANISM (applyState(0) inside
// resetFilter()) is absent here. The CONSEQUENCE is not.
//
// `initializeFilter()` runs at CONFIGURE and starts the heartbeat.
// `map_update_thread_` is not created until ACTIVATE (costmap_2d_ros.cpp:314).
// So on every ordinary bringup there is a window in which the filter publishes
// while it has never been driven, has read no mask and has issued no set --
// and `enforcementToken()` computes `yes` for it, because nothing has failed.
//
// A consumer cannot distinguish that from a zone confirmed on every target.
// Neither can it distinguish it from the STOPPED case, which this package has
// already ruled must not read as safety -- and to a consumer they are the same
// fact: NOBODY IS WATCHING.
// ===========================================================================
TEST_F(MaintainerFindings, MF1_NeverDrivenMustNotReadAsEnforced)
{
  ASSERT_TRUE(build(kZoneCell));
  ASSERT_TRUE(waitActive()) << "the latched mask must arrive";

  // Deliberately do NOT tick the costmap. This is the configure->activate gap.
  const size_t m = recorder_->mark();
  settle(500ms);  // ~5 heartbeats at liveness_period 0.1
  const auto samples = recorder_->since(m);

  ASSERT_FALSE(samples.empty())
    << "the heartbeat must be publishing, or this test proves nothing";

  for (const auto & s : samples) {
    EXPECT_NE(get(s, "enforced"), "yes")
      << "published `enforced: yes` having never been driven once: " << render(s);
    EXPECT_NE(get(s, "watching"), "yes")
      << "published `watching: yes` having never been driven once: " << render(s);
    EXPECT_NE(s.level, diagnostic_msgs::msg::DiagnosticStatus::OK)
      << "published level OK having never been driven once: " << render(s);
  }
}

// ===========================================================================
// MF-2 -- NEGATIVE CONTROL for MF-1.
//
// A filter that IS being driven, inside a zone, with the set confirmed, must
// read `yes`. Without this, MF-1 could be satisfied by a filter that never says
// `yes` at all -- a detector that cannot be wrong because it never asserts
// anything, which is not the property being bought.
// ===========================================================================
TEST_F(MaintainerFindings, MF2_NegativeControl_ADrivenConfirmedZoneStillReadsYes)
{
  ASSERT_TRUE(build(kZoneCell));
  ASSERT_TRUE(waitActive());

  const auto deadline = std::chrono::steady_clock::now() + 8000ms;
  bool saw_yes = false;
  while (std::chrono::steady_clock::now() < deadline && !saw_yes) {
    const size_t m = recorder_->mark();
    tickCostmap();
    settle(100ms);
    for (const auto & s : recorder_->since(m)) {
      if (get(s, "enforced") == "yes" && get(s, "watching") == "yes") {saw_yes = true;}
    }
  }
  EXPECT_TRUE(saw_yes)
    << "a driven, confirmed, in-zone filter must be able to report `yes`; "
       "if it cannot, MF-1 is passing for the wrong reason";
  EXPECT_DOUBLE_EQ(target_node_->liveSpeed(), kZoneSpeed)
    << "and the zone's value must actually be on the target";
}

// ===========================================================================
// MF-3 -- THE ROUTINE CLEAR, ASSERTED INSIDE THE WINDOW.
//
// The consequence of his first finding, as recorded from his review:
//
//   "A clear_entirely_<costmap> -- which the default BT fires as a routine
//    recovery -- restores nominal defaults to every target while the robot is
//    inside the zone, then blocks in process() at !filter_mask_ until the
//    latched mask is redelivered on a new subscription. For that window the
//    speed limit is lifted and the event topic still announces state N."
//
// Here the mechanism differs and the direction of the lie inverts: this filter
// does NOT restore, so the ZONE'S VALUE IS STILL ON THE TARGET -- while the
// filter, having cleared current_state_ and dropped the mask, publishes
// "Outside any zone; nominal defaults are in force."
//
// The claim and the world disagree, and the surface renders only the claim.
// Where the robot's account and the observed state can differ, both are
// published and neither is collapsed -- that is this package's stated design
// constraint, and this is the case that tests it.
// ===========================================================================
TEST_F(MaintainerFindings, MF3_RoutineClearMustNotClaimNominalDefaultsItNeverRestored)
{
  ASSERT_TRUE(build(kZoneCell));
  ASSERT_TRUE(waitActive());

  // Get genuinely into the zone and confirmed.
  const auto deadline = std::chrono::steady_clock::now() + 8000ms;
  while (std::chrono::steady_clock::now() < deadline &&
    target_node_->liveSpeed() != kZoneSpeed)
  {
    tickCostmap();
    settle(50ms);
  }
  ASSERT_DOUBLE_EQ(target_node_->liveSpeed(), kZoneSpeed)
    << "precondition: the zone must be in force on the target";

  // THE WINDOW OPENS HERE. No wait-for-active after this line.
  const size_t m = recorder_->mark();
  routineClearNoWait();
  settle(400ms);
  const auto samples = recorder_->since(m);

  ASSERT_FALSE(samples.empty())
    << "the heartbeat must speak inside the window, or nothing can be asserted";

  // The world, measured independently of the filter's account of it.
  const double live = target_node_->liveSpeed();
  ASSERT_DOUBLE_EQ(live, kZoneSpeed)
    << "this filter does not restore in resetFilter(), so the zone's value must "
       "still be on the target -- if this fails the premise has changed";

  for (const auto & s : samples) {
    EXPECT_EQ(s.message.find("nominal defaults are in force"), std::string::npos)
      << "claimed nominal defaults are in force while the target still holds "
      << live << ": " << render(s);
    EXPECT_NE(get(s, "enforced"), "yes")
      << "claimed `enforced: yes` in the window after a routine clear, with no "
         "mask, no configuration applied and nothing confirmed: " << render(s);
  }
}

// ===========================================================================
// MF-4 -- THE CLEAR ITSELF IS NEVER REPORTED.
//
// His second finding is an ORDERING one: `state_event_pub_` is destroyed BEFORE
// the state change, so the change is silent on the publish surface.
//
// The same ordering is here, at src/zone_parameter_filter.cpp:1082-1089 (both
// publishers destroyed) followed by :1092-1143 (current_state_,
// state_initialized_, mask_state_ and unconfirmed_targets_ all mutated).
// Nothing announces the transition, because the thing that would announce it
// was destroyed one screen earlier.
//
// Silence is a communication failure, not a reporting gap. A bridge that goes
// quiet while its last message stays on the screen is communicating something
// false, carefully.
// ===========================================================================
TEST_F(MaintainerFindings, MF4_TheRoutineClearIsAnnouncedSomewhere)
{
  ASSERT_TRUE(build(kZoneCell));
  ASSERT_TRUE(waitActive());

  const auto deadline = std::chrono::steady_clock::now() + 8000ms;
  while (std::chrono::steady_clock::now() < deadline &&
    target_node_->liveSpeed() != kZoneSpeed)
  {
    tickCostmap();
    settle(50ms);
  }
  ASSERT_DOUBLE_EQ(target_node_->liveSpeed(), kZoneSpeed);

  const size_t m = recorder_->mark();
  routineClearNoWait();
  settle(400ms);
  const auto samples = recorder_->since(m);
  ASSERT_FALSE(samples.empty());

  bool announced = false;
  for (const auto & s : samples) {
    const std::string ev = get(s, "event");
    if (ev != "<absent>" &&
      (ev.find("clear") != std::string::npos ||
      ev.find("reset") != std::string::npos ||
      ev.find("reload") != std::string::npos))
    {
      announced = true;
    }
  }
  EXPECT_TRUE(announced)
    << "the filter was reset out from under its own configuration and no message "
       "said so; a consumer sees only a zone_state that silently became 0";
}

// ===========================================================================
// MF-5 -- A SET IN FLIGHT AT THE CLEAR IS FORGOTTEN, AND `pending` BECOMES
// `yes` WITHOUT A SINGLE CONFIRMATION EVER ARRIVING.
//
// His fourth finding is that the in-flight bookkeeping is sampled at the wrong
// moment, so the restore's own sets never arm the re-apply. This package has no
// re-apply mechanism at all -- but it has the same bookkeeping, and it does
// something stronger: resetFilter() CLEARS `pending_sets_` and
// `unconfirmed_targets_` (:1140-1141) while the requests are still on the wire.
//
// So the filter forgets a set it issued, can never learn its outcome, and the
// token it publishes moves from `pending` to `yes` -- the reassuring value,
// reached by discarding the evidence that refuted it. That is the exact
// success-shaped value this package's own header says it exists to abolish.
//
// The target here does not exist, so the set is genuinely unanswerable. It is
// the honest form of "in flight at the moment of the clear".
// ===========================================================================
TEST_F(MaintainerFindings, MF5_ASetInFlightAtTheClearMustNotResolveToYes)
{
  ASSERT_TRUE(build(kZoneCell, /*target_exists=*/false));
  ASSERT_TRUE(waitActive());

  // Drive into the zone: the set is issued at a node nobody is running.
  bool saw_pending = false;
  const auto deadline = std::chrono::steady_clock::now() + 8000ms;
  while (std::chrono::steady_clock::now() < deadline && !saw_pending) {
    const size_t m = recorder_->mark();
    tickCostmap();
    settle(100ms);
    for (const auto & s : recorder_->since(m)) {
      if (get(s, "enforced") == "pending") {saw_pending = true;}
    }
  }
  ASSERT_TRUE(saw_pending)
    << "precondition: an unanswerable set must show as `pending` first";

  // THE WINDOW. The set is still outstanding when the clear lands.
  const size_t m = recorder_->mark();
  routineClearNoWait();
  settle(400ms);
  const auto samples = recorder_->since(m);
  ASSERT_FALSE(samples.empty());

  for (const auto & s : samples) {
    EXPECT_NE(get(s, "enforced"), "yes")
      << "a set that was outstanding at the clear was dropped, and the token "
         "resolved to `yes` without any confirmation ever arriving: " << render(s);
  }
}

// ===========================================================================
// MF-6 -- THE `unknown` VALUE HAS AN ASSERTION.
//
// `unknown` was introduced as a BREAKING change to the documented consumer
// contract and had ZERO assertions anywhere in the suite -- measured
// 2026-09-06, `grep -rn '"unknown"' test/` returned nothing. A breaking change
// nothing checks is a claim, not a feature.
//
// This is not one of his findings. It is the same CLASS as the ones he keeps
// finding: a value that exists in the code and in the changelog and in no
// oracle.
// ===========================================================================
TEST_F(MaintainerFindings, MF6_SilenceReportsUnknownAndNotYes)
{
  ASSERT_TRUE(
    build(
      kZoneCell, true, /*liveness_period=*/0.1,
      /*silence_timeout=*/0.3));
  ASSERT_TRUE(waitActive());

  // Drive it, then stop driving it. The executor keeps spinning.
  for (int i = 0; i < 6; ++i) {tickCostmap(); settle(50ms);}

  const size_t m = recorder_->mark();
  settle(1200ms);  // >> silence_timeout
  const auto samples = recorder_->since(m);
  ASSERT_FALSE(samples.empty());

  bool saw_unknown = false;
  for (const auto & s : samples) {
    if (get(s, "enforced") == "unknown") {saw_unknown = true;}
  }
  EXPECT_TRUE(saw_unknown)
    << "the costmap stopped and `enforced` never reported `unknown`";

  const auto & last = samples.back();
  EXPECT_EQ(get(last, "watching"), "NO") << render(last);
  EXPECT_EQ(last.level, diagnostic_msgs::msg::DiagnosticStatus::STALE) << render(last);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
