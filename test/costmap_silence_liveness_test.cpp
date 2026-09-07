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

// ============================================================================
// ⚑ THE ORACLE MUST DISTINGUISH **STOPPED** FROM **RUNNING-AND-QUIET**.
//
// Both look identical on the old code: the filter publishes only when something
// CHANGES, so a healthy filter with nothing to say and a filter that is no
// longer running produce the same observable -- an unchanging `OK` on the wire.
// An oracle that asks "did anything arrive?" cannot tell them apart, and an
// oracle that cannot tell them apart has measured nothing.
//
// So every case below drives TWO ARMS that differ in exactly one thing -- is
// updateCosts() still being called -- and asserts that the observable DIFFERS.
// A test asserting only "messages arrive" would pass both arms and prove
// nothing; ARM_A_AND_B_ARE_NOT_SEPARATED_BY_MESSAGE_COUNT states that failure
// explicitly rather than leaving it implied.
//
// Topology is production's, per the lesson recorded at f49b92e: the executor
// spins the node on ITS OWN THREAD (as controller_server.cpp:72 and
// planner_server.cpp:78 do with nav2::NodeThread), while updateCosts() is
// driven from a DIFFERENT thread -- which is the whole point, since the claim
// under test is that one of those two threads can stop without the other.
// ============================================================================

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
#include "hubot/nav2_compat.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "hubot/zone_parameter_filter.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";

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
  : Node("liveness_info_pub")
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
  : Node("liveness_mask_pub")
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }
  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

class TargetNode : public rclcpp::Node
{
public:
  TargetNode()
  : rclcpp::Node("liveness_target_node")
  {
    declare_parameter("speed", 1.0);
  }
};

// ⚑ THE OBSERVER. It records EVERY message with its own arrival wall-time, so
// the test can reason about the SEQUENCE, not only the last value. A last-value
// read cannot see a transition; a count cannot see a direction.
//
// It lives on its own node and its own executor thread: the observer must not
// be driven by the thing it observes. That is the f49b92e lesson in its general
// form -- an oracle co-driven with the subject reports the subject's liveness
// as its own.
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
  : Node("liveness_decision_recorder"), t0_(std::chrono::steady_clock::now())
  {
    subscriber_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "zone_decision", rclcpp::QoS(200),
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

class LivenessTest : public ::testing::Test
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

  // liveness_period <= 0 disables the mechanism -- the NEGATIVE CONTROL.
  // target_exists=false routes the zone's set at a node nobody is running, so
  // the future never becomes ready: the F-2 silence case.
  bool build(
    double liveness_period, double silence_timeout, double set_timeout,
    int8_t mask_fill, bool target_exists)
  {
    std::vector<rclcpp::Parameter> cfg = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
      rclcpp::Parameter(fp("liveness_period"), liveness_period),
      rclcpp::Parameter(fp("costmap_silence_timeout"), silence_timeout),
      rclcpp::Parameter(fp("set_parameters_timeout"), set_timeout),
      rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow"}),
      rclcpp::Parameter(fp("slow.id"), static_cast<int64_t>(3)),
      rclcpp::Parameter(fp("slow.setpoints"), std::vector<std::string>{"cap"}),
      rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"cap"}),
    };
    const std::string target =
      target_exists ? "liveness_target_node" : "a_node_that_is_not_running";
    addEntry(cfg, "slow.cap", target, "speed", rclcpp::ParameterValue(0.2));
    addEntry(cfg, "nominal_defaults.cap", target, "speed", rclcpp::ParameterValue(1.0));

    rclcpp::NodeOptions opts;
    opts.parameter_overrides(cfg);

    if (target_exists) {
      target_node_ = std::make_shared<TargetNode>();
    }
    node_ = std::make_shared<hubot::HostNode>("liveness_host", "", opts);
    recorder_ = std::make_shared<DecisionRecorder>();

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = hubot::createTransformBuffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);

    filter_ = std::make_shared<hubot::ZoneParameterFilter>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>();
    mask_pub_ = std::make_shared<MaskPublisher>(make_mask(4, 4, mask_fill));

    startSpinning();

    // Wait for the latched info+mask to arrive through the spinning executors.
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!filter_->isActive()) {
      if (std::chrono::steady_clock::now() > deadline) {return false;}
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  // ⚑ PRODUCTION TOPOLOGY. Every node spins on a thread that is NOT the test
  // thread, exactly as nav2::NodeThread does for the costmap node
  // (controller_server.cpp:72). The costmap update loop -- driven below from
  // the test thread -- is therefore genuinely independent of the executor, and
  // stopping one does not stop the other. If they shared a thread this test
  // could not pose its question at all.
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

  // ARM A: the costmap keeps ticking and NOTHING CHANGES.
  void driveQuietly(std::chrono::milliseconds duration)
  {
    const auto end = std::chrono::steady_clock::now() + duration;
    while (std::chrono::steady_clock::now() < end) {
      tickCostmap();
      std::this_thread::sleep_for(50ms);
    }
  }

  // ARM B: the costmap STOPS. The executor keeps spinning on its own thread.
  void stayStopped(std::chrono::milliseconds duration)
  {
    std::this_thread::sleep_for(duration);
  }

  std::shared_ptr<hubot::HostNode> node_;
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

namespace
{
std::string get(const DecisionRecorder::Sample & s, const std::string & k)
{
  auto it = s.values.find(k);
  return it == s.values.end() ? std::string("<absent>") : it->second;
}
}  // namespace

// ===========================================================================
// CASE 1 -- THE DISCRIMINATION. Same filter, same configuration, same steady
// state; the ONLY difference is whether updateCosts() is still being called.
// ===========================================================================
TEST_F(LivenessTest, StoppedIsDistinguishableFromRunningAndQuiet)
{
  ASSERT_TRUE(build(0.2, 0.5, 5.0, /*mask_fill=*/3, /*target_exists=*/true));

  // Reach a steady, quiet, confirmed state first.
  for (int i = 0; i < 20; ++i) {
    tickCostmap();
    std::this_thread::sleep_for(25ms);
  }

  // ---- ARM A: running and quiet -------------------------------------------
  const size_t mark_a = recorder_->mark();
  driveQuietly(2000ms);
  const auto arm_a = recorder_->since(mark_a);

  // ---- ARM B: stopped ------------------------------------------------------
  const size_t mark_b = recorder_->mark();
  stayStopped(2000ms);
  const auto arm_b = recorder_->since(mark_b);

  // -- The trap, stated as an assertion rather than left implied. If the only
  //    oracle were "did messages arrive", BOTH arms pass and nothing has been
  //    measured. This case exists because that oracle is worthless here.
  ASSERT_GT(arm_a.size(), 0u) << "ARM A produced no liveness signal at all";
  ASSERT_GT(arm_b.size(), 0u)
    << "ARM B produced NO signal -- silence is still reading as safety, which "
       "is the defect this test exists for";
  EXPECT_GT(arm_a.size(), 0u) << "ARM_A_AND_B_ARE_NOT_SEPARATED_BY_MESSAGE_COUNT";
  EXPECT_GT(arm_b.size(), 0u) << "ARM_A_AND_B_ARE_NOT_SEPARATED_BY_MESSAGE_COUNT";

  // -- ARM A: every sample says it IS watching, and the age never grows past
  //    the silence timeout.
  for (const auto & s : arm_a) {
    EXPECT_EQ(get(s, "watching"), "yes")
      << "a RUNNING filter reported that it was not watching";
    EXPECT_LT(std::stod(get(s, "costmap_age_s")), 0.5)
      << "a RUNNING filter reported a stale costmap age";
  }
  EXPECT_NE(arm_a.back().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);

  // -- ARM B: it flips to NOT watching, and says so.
  bool saw_not_watching = false;
  for (const auto & s : arm_b) {
    if (get(s, "watching") == "NO") {saw_not_watching = true;}
  }
  EXPECT_TRUE(saw_not_watching)
    << "the costmap stopped and the filter never said so";
  EXPECT_EQ(get(arm_b.back(), "watching"), "NO");
  EXPECT_EQ(arm_b.back().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
  EXPECT_NE(get(arm_b.back(), "enforced"), "yes")
    << "a filter that is not watching still reported the zone as enforced";

  // -- THE DIRECTION, not merely the difference: the reported age must GROW
  //    while stopped. A field that merely flips could be flipping for any
  //    reason; one that counts up is measuring the thing it names.
  ASSERT_GE(arm_b.size(), 2u);
  EXPECT_GT(
    std::stod(get(arm_b.back(), "costmap_age_s")),
    std::stod(get(arm_b.front(), "costmap_age_s")))
    << "costmap_age_s did not advance while the costmap was stopped";

  // -- THE HEARTBEAT ITSELF IS ALIVE. report_seq strictly increasing across
  //    BOTH arms is what separates "stopped costmap, live filter" from
  //    "everything is gone" -- the third state, which no field inside a
  //    message can ever report, and which only the ABSENCE of a promised
  //    message can.
  std::vector<long long> seqs;
  for (const auto & s : arm_a) {seqs.push_back(std::stoll(get(s, "report_seq")));}
  for (const auto & s : arm_b) {seqs.push_back(std::stoll(get(s, "report_seq")));}
  for (size_t i = 1; i < seqs.size(); ++i) {
    EXPECT_GT(seqs[i], seqs[i - 1]) << "report_seq did not advance at index " << i;
  }

  // -- THE SELF-DESCRIBING EXPIRY. Every message must carry the promise a
  //    reader needs when the whole node is gone and no further message can
  //    ever arrive.
  for (const auto & s : arm_b) {
    EXPECT_NE(get(s, "valid_for_s"), "<absent>");
    EXPECT_GT(std::stod(get(s, "valid_for_s")), 0.0);
    EXPECT_NE(get(s, "report_period_s"), "<absent>");
  }
}

// ===========================================================================
// CASE 2 -- THE NEGATIVE CONTROL. With the mechanism disabled the harness must
// REPRODUCE THE DEFECT: arm B goes silent and the last thing on the wire still
// says the zone is enforced. If this case fails, case 1 is passing for some
// reason other than the mechanism under test and proves nothing.
// ===========================================================================
TEST_F(LivenessTest, NegativeControl_WithoutTheMechanismSilenceStillReadsAsSafety)
{
  ASSERT_TRUE(build(-1.0, 0.5, 5.0, /*mask_fill=*/3, /*target_exists=*/true));

  for (int i = 0; i < 20; ++i) {
    tickCostmap();
    std::this_thread::sleep_for(25ms);
  }

  const size_t mark = recorder_->mark();
  const auto before = recorder_->since(0);
  ASSERT_GT(before.size(), 0u) << "the filter never published at all";
  const auto last_before = before.back();

  stayStopped(2000ms);
  const auto during = recorder_->since(mark);

  EXPECT_EQ(during.size(), 0u)
    << "the mechanism is disabled, yet something published during the silence -- "
       "case 1 may be passing for a reason other than the liveness timer";
  EXPECT_EQ(get(last_before, "enforced"), "yes")
    << "the defect being reproduced is that the LAST value stands and reads as safe";
  EXPECT_EQ(last_before.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
}

// ===========================================================================
// CASE 3 -- THE HOLE IS NOT MERELY ANNOUNCED, IT IS CLOSED. AoU-5 stated that
// with the costmap stopped, `pending` never resolves and the set_parameters
// deadline never fires. checkPendingParameterUpdates() needs the clock and the
// futures and NOTHING from the costmap, so a driver that is not the costmap can
// run it. This case proves the deadline fires with ZERO further updateCosts()
// calls -- which is the sentence on hubot's face going false.
// ===========================================================================
TEST_F(LivenessTest, DeadlineFiresWhileTheCostmapIsStopped)
{
  // The zone's target is a node nobody is running: the set is issued and never
  // answered. set_parameters_timeout is 0.5 s.
  ASSERT_TRUE(build(0.2, 10.0, 0.5, /*mask_fill=*/3, /*target_exists=*/false));

  // ONE tick: enter the zone, issue the set. Then never tick again.
  tickCostmap();
  const size_t mark = recorder_->mark();

  stayStopped(2500ms);
  const auto after = recorder_->since(mark);

  bool saw_no = false;
  bool saw_deadline_event = false;
  for (const auto & s : after) {
    if (get(s, "enforced") == "NO") {saw_no = true;}
    if (get(s, "event").find("no answer within") != std::string::npos) {
      saw_deadline_event = true;
    }
  }
  EXPECT_TRUE(saw_no)
    << "the set was never answered and the costmap never ticked again, yet the "
       "filter never resolved `pending` to `NO`";
  EXPECT_TRUE(saw_deadline_event)
    << "the set_parameters deadline did not fire without a costmap tick";
}

// ===========================================================================
// CASE 4 -- THE NEGATIVE CONTROL FOR CASE 3. Same silence, same unanswered set,
// mechanism disabled: `pending` must stand forever, exactly as AoU-5 said.
// ===========================================================================
TEST_F(LivenessTest, NegativeControl_DeadlineNeverFiresWithoutTheMechanism)
{
  ASSERT_TRUE(build(-1.0, 10.0, 0.5, /*mask_fill=*/3, /*target_exists=*/false));

  tickCostmap();
  const size_t mark = recorder_->mark();

  stayStopped(2500ms);
  const auto after = recorder_->since(mark);

  for (const auto & s : after) {
    EXPECT_NE(get(s, "enforced"), "NO")
      << "with the mechanism disabled the deadline must NOT be able to fire";
  }
  const auto all = recorder_->since(0);
  ASSERT_GT(all.size(), 0u);
  EXPECT_EQ(get(all.back(), "enforced"), "pending")
    << "the reproduced defect is that `pending` stands forever";
}

// ===========================================================================
// CASE 5 -- RECOVERY. A filter that says NOT WATCHING and never takes it back
// has replaced one stuck value with another. The costmap resuming must be
// announced too.
// ===========================================================================
TEST_F(LivenessTest, ResumingTheCostmapIsAnnounced)
{
  ASSERT_TRUE(build(0.2, 0.5, 5.0, /*mask_fill=*/3, /*target_exists=*/true));

  for (int i = 0; i < 20; ++i) {
    tickCostmap();
    std::this_thread::sleep_for(25ms);
  }
  stayStopped(1500ms);

  const size_t mark = recorder_->mark();
  driveQuietly(1500ms);
  const auto after = recorder_->since(mark);

  ASSERT_GT(after.size(), 0u);
  EXPECT_EQ(get(after.back(), "watching"), "yes")
    << "the costmap resumed and the filter never took back `not watching`";
  EXPECT_NE(after.back().level, diagnostic_msgs::msg::DiagnosticStatus::STALE);
}

// ===========================================================================
// CASE 6 -- ⚑ THE TIMER MUST SURVIVE reset(), CONCURRENTLY.
//
// This case exists because the risk it tests was found by asking what the new
// mechanism could BREAK, not by asking whether it works. CostmapFilter::reset()
// is `resetFilter(); initializeFilter(...)` (costmap_filter.cpp:103-108) and it
// arrives on the COSTMAP UPDATE THREAD, holding the filter mutex, while it
// destroys and rebuilds node entities -- including, now, a timer whose callback
// wants that same mutex on the EXECUTOR THREAD. ClearEntireCostmap reaches this
// path from seven of nav2's default behaviour trees, so it is an ordinary
// event, not an exotic one.
//
// The existing reset coverage (HUBOT_D1/D2) drives everything from one thread
// with spin_some(), so it cannot pose this question at all. If reset and the
// tick can deadlock, this test hangs and ctest kills it -- a hang IS the
// failure signal here, which is why the case is worth its seconds.
// ===========================================================================
TEST_F(LivenessTest, ResetWhileTheTimerIsLiveNeitherHangsNorLosesTheHeartbeat)
{
  ASSERT_TRUE(build(0.05, 0.5, 5.0, /*mask_fill=*/3, /*target_exists=*/true));

  // Hammer reset from the test thread (standing in for the costmap update
  // thread) while the executor thread dispatches a 20 Hz timer.
  for (int i = 0; i < 25; ++i) {
    tickCostmap();
    filter_->resetFilter();
    filter_->initializeFilter(kInfoTopic);
    std::this_thread::sleep_for(20ms);
  }

  // Getting here at all is most of the assertion. The rest: the heartbeat must
  // be alive on the OTHER side of all that, or reset has quietly killed the
  // detector and left a filter that looks healthy because it can no longer
  // speak.
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (!filter_->isActive()) {
    if (std::chrono::steady_clock::now() > deadline) {break;}
    std::this_thread::sleep_for(10ms);
  }
  const size_t mark = recorder_->mark();
  driveQuietly(600ms);
  const auto after = recorder_->since(mark);

  EXPECT_GT(after.size(), 0u)
    << "the heartbeat did not survive resetFilter()/initializeFilter(); a "
       "ClearEntireCostmap recovery would silently disable the liveness signal";
  if (!after.empty()) {
    EXPECT_EQ(get(after.back(), "watching"), "yes");
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
