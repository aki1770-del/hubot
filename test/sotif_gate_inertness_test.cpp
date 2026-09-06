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
// ⚑ THE CLASS ORACLE — SOTIF (ISO 21448), NOT ISO 26262.
//
// Authored by FSE 2026-09-06. QM/advisory. No ASIL is claimed, rated or
// implied. See doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md for the analysis this
// file is the executable half of.
//
// ⚑ EXECUTED 2026-09-06. Image `nav2-lyrical-main-compat:latest`, `--network=none`,
// nav2_costmap_2d 1.5.1 with ZONE_PARAMETER_FILTER = 4. Build exit 0 in 29.9 s,
// first attempt, zero errors.
//
//   SC-1  RED   -- `enforced=yes watching=yes level=OK costmap_age_s=1.506290`
//                  from a costmap stopped 1.5 s, with the detector configured off
//   SC-2  GREEN -- negative control holds, so SC-1's red is not vacuous
//   SC-3  RED   -- `valid_for_s=2.500000` against a 2.0 s silence budget, at defaults
//   SC-4  GREEN -- the never-driven branch already reported `unknown` + STALE;
//                  it was UNASSERTED, not wrong
//   SC-5  RED   -- deadline arm 0 messages, control arm 22, incompatible_qos fired
//   SC-6  GREEN -- tripwire armed under AoU-S1
//
// Full suite re-run the same session: 33 gtest cases, 2 failures, both of them
// SC-1 and SC-3. NO PRE-EXISTING CASE REGRESSED.
//
// ⚑ THIS HEADER SAID "THIS FILE HAS NEVER BEEN COMPILED OR RUN… the whole package
// cannot build today" UNTIL THE ABOVE WAS MEASURED, and the sentence is recorded
// here rather than deleted because it was FALSE WHEN WRITTEN. The instrument was
// `ls /opt/ros` and `which colcon` on the HOST; docker, the image and a built
// workspace were all present. A method that could not have surfaced the
// counter-example has measured nothing (Vision 77). FBR had corrected this exact
// error in another seat's file the day before.
//
// ⚑ STILL UNVERIFIED, and not cleared: SC-5's result is true of the rmw in this
// image only -- DEADLINE Request/Offered handling is implementation-specific.
// And every case here drives updateCosts() BY HAND from the test thread, so the
// thread-independence claim the whole liveness design rests on is simulated, not
// exercised. See doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md §6.
//
// ---------------------------------------------------------------------------
// WHY A CLASS ORACLE AND NOT A FIFTH CASE.
//
// Four generations of one defect have been fixed in this package:
//   1. `enforced: yes` on the cycle the sets were merely ISSUED;
//   2. `enforced: yes` at STARTUP, having configured nothing;
//   3. `enforced: yes` during costmap SILENCE;
//   4. `enforced: yes` BEFORE THE FIRST TICK, from a filter never driven.
//
// The package's own diagnosis (zone_parameter_filter.hpp:306-333) is right and
// is the reason this file exists: *"Every fix went into a caller and every new
// caller arrived without it. So this one goes into the VALUE."* `notWatching()`
// is that value-level gate.
//
// ⚑ BUT A GATE HAS INPUTS, AND THIS ORACLE IS ABOUT THE INPUTS. `notWatching()`
// reads `costmap_silent_`, which is computed at zone_parameter_filter.cpp:923
// as `(costmap_silence_timeout_ > 0.0) && (age > costmap_silence_timeout_)`.
// Set that parameter to zero — a DOCUMENTED configuration, README:440, *"keep
// the heartbeat but never report a stopped costmap"* — and `costmap_silent_`
// can never become true. After one tick `ever_processed_` is true, so
// `notWatching()` is false forever, and a filter whose costmap died an hour ago
// publishes `watching: yes`, `enforced: yes`, level `OK`.
//
// THAT IS GENERATION 3 EXACTLY, REINSTATED BY ONE LINE OF YAML. The value-level
// gate does not cover it, because the gate cannot report its own inertness.
//
// So the invariant asserted here is not about any one generation:
//
//   ⚑ SOTIF-CLASS-1 — NO CONFIGURATION OF THE LIVENESS GATE MAY PRODUCE A
//     REASSURING VALUE FROM A FILTER THAT NOTHING IS DRIVING. A detector an
//     integrator switched off must report that it is off, in the VALUE, and
//     must never report the reassuring value instead. Disabling a detector
//     removes a measurement; it does not create a good one.
//
// A test suite that adds one case per generation is a suite that will always be
// one generation behind. This one is a property over the configuration space.
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

using namespace std::chrono_literals;

namespace
{
constexpr char kFilterName[] = "zone_parameter_filter";
constexpr char kInfoTopic[] = "costmap_filter_info";
constexpr char kMaskTopic[] = "mask";
constexpr int8_t kZoneCell = 3;

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
  explicit InfoPublisher(const std::string & ns = "")
  : Node("sotif_info_pub", ns)
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
  MaskPublisher(const nav_msgs::msg::OccupancyGrid & mask, const std::string & ns = "")
  : Node("sotif_mask_pub", ns)
  {
    publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      kMaskTopic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(mask);
  }
  ~MaskPublisher() override {publisher_.reset();}

private:
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

/// A target that actually answers, so `pending` can resolve to `yes` and the
/// negative control has something real to confirm against.
class TargetNode : public rclcpp::Node
{
public:
  explicit TargetNode(const std::string & ns = "")
  : Node("sotif_target_node", ns)
  {
    declare_parameter("speed", 1.0);
  }
  double liveSpeed() {return get_parameter("speed").as_double();}
};

class DecisionRecorder : public rclcpp::Node
{
public:
  struct Sample
  {
    uint8_t level;
    std::map<std::string, std::string> values;
    std::string message;
  };

  explicit DecisionRecorder(const std::string & ns = "")
  : Node("sotif_decision_recorder", ns)
  {
    subscriber_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "zone_decision", rclcpp::QoS(200),
      [this](const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(m_);
        for (const auto & st : msg->status) {
          Sample s;
          s.level = st.level;
          s.message = st.message;
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
  std::string out = "level=" + std::to_string(static_cast<int>(s.level));
  for (const auto & [k, v] : s.values) {out += " " + k + "=" + v;}
  return out + " | " + s.message;
}
}  // namespace

class SotifGateInertness : public ::testing::Test
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

  /// `silence_timeout <= 0` is the DOCUMENTED disable (README:440). It is the
  /// configuration under test, not an abuse of the interface.
  bool build(
    double liveness_period, double silence_timeout, int8_t mask_fill = kZoneCell,
    const std::string & host_ns = "", const std::string & target_ns = "",
    const std::string & topic_ns = "")
  {
    std::vector<rclcpp::Parameter> cfg = {
      rclcpp::Parameter(fp("filter_info_topic"), std::string(kInfoTopic)),
      rclcpp::Parameter(fp("transform_tolerance"), 0.5),
      rclcpp::Parameter(fp("liveness_period"), liveness_period),
      rclcpp::Parameter(fp("costmap_silence_timeout"), silence_timeout),
      rclcpp::Parameter(fp("set_parameters_timeout"), 5.0),
      rclcpp::Parameter(fp("states"), std::vector<std::string>{"slow"}),
      rclcpp::Parameter(fp("slow.id"), static_cast<int64_t>(3)),
      rclcpp::Parameter(fp("slow.setpoints"), std::vector<std::string>{"cap"}),
      rclcpp::Parameter(fp("nominal_defaults"), std::vector<std::string>{"cap"}),
    };
    addEntry(cfg, "slow.cap", "sotif_target_node", "speed", rclcpp::ParameterValue(0.2));
    addEntry(cfg, "nominal_defaults.cap", "sotif_target_node", "speed",
      rclcpp::ParameterValue(1.0));

    rclcpp::NodeOptions opts;
    opts.parameter_overrides(cfg);

    target_node_ = std::make_shared<TargetNode>(target_ns);
    node_ = std::make_shared<nav2::LifecycleNode>("sotif_host", host_ns, opts);
    recorder_ = std::make_shared<DecisionRecorder>(topic_ns);

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    tf_buffer_ = nav2::create_transform_buffer(node_);
    tf_buffer_->setUsingDedicatedThread(true);

    filter_ = std::make_shared<hubot::ZoneParameterFilter>();
    filter_->initialize(layers_.get(), kFilterName, tf_buffer_.get(), node_, nullptr);
    filter_->initializeFilter(kInfoTopic);

    info_pub_ = std::make_shared<InfoPublisher>(topic_ns);
    mask_pub_ = std::make_shared<MaskPublisher>(make_mask(4, 4, mask_fill), topic_ns);

    startSpinning();

    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!filter_->isActive()) {
      if (std::chrono::steady_clock::now() > deadline) {return false;}
      std::this_thread::sleep_for(10ms);
    }
    return true;
  }

  /// Production topology: the node spins on its OWN thread, the costmap update
  /// loop is driven from the test thread. The two must be able to stop
  /// independently or this file cannot pose its question.
  void startSpinning()
  {
    exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec_->add_node(node_->get_node_base_interface());
    exec_->add_node(recorder_);
    exec_->add_node(info_pub_);
    exec_->add_node(mask_pub_);
    exec_->add_node(target_node_);
    spin_thread_ = std::thread([this]() {exec_->spin();});
  }

  void stopSpinning()
  {
    if (exec_) {exec_->cancel();}
    if (spin_thread_.joinable()) {spin_thread_.join();}
    exec_.reset();
  }

  void tickCostmap(double x = 1.5, double y = 1.5)
  {
    nav2_costmap_2d::Costmap2D costmap(4, 4, 1.0, 0.0, 0.0, 0);
    double mnx = 0, mny = 0, mxx = 0, mxy = 0;
    filter_->updateBounds(x, y, 0.0, &mnx, &mny, &mxx, &mxy);
    filter_->updateCosts(costmap, 0, 0, 4, 4);
  }

  void driveUntilConfirmed(std::chrono::milliseconds budget)
  {
    const auto end = std::chrono::steady_clock::now() + budget;
    while (std::chrono::steady_clock::now() < end) {
      tickCostmap();
      std::this_thread::sleep_for(25ms);
    }
  }

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
// SC-1 — ⚑ THE FIFTH GENERATION. EXPECTED RED ON CURRENT SOURCE.
//
// `costmap_silence_timeout: 0.0` is documented at README:440 as *"keep the
// heartbeat but never report a stopped costmap"*, and it does exactly that:
// zone_parameter_filter.cpp:923 short-circuits on `costmap_silence_timeout_ >
// 0.0`, so `costmap_silent_` stays false whatever the age. The heartbeat then
// publishes, once per period, forever, from a filter nothing is driving:
//   watching: yes · enforced: yes · level OK · "Zone 3 is in force."
//
// A consumer applying the package's own whitelist rule — PROCEED ONLY ON
// `enforced: yes` — proceeds. The zone's limits may not be on anything.
//
// The integrator did not ask for that. They asked not to be told about a
// stopped costmap; they were given an affirmative claim that it is running.
// ⚑ SUPPRESSING A WARNING AND ASSERTING SAFETY ARE DIFFERENT ACTS, and the
// wire cannot presently tell them apart.
//
// THE FIX BELONGS IN THE VALUE, for the reason the header already gives:
//     bool notWatching() const
//     {return !ever_processed_ || costmap_silent_ || costmap_silence_timeout_ <= 0.0;}
// ⚑ FSE does not hold the pen on src/. This case states the invariant; CPP
// owns whether and how the value changes.
// ===========================================================================
TEST_F(SotifGateInertness, SC1_ADisabledSilenceDetectorMustNotReportSafety)
{
  ASSERT_TRUE(build(/*liveness_period=*/0.1, /*silence_timeout=*/0.0));

  // Get genuinely driven and confirmed first, so `ever_processed_` is true and
  // this case is about the DETECTOR, not about the never-driven branch MF-1
  // already covers.
  driveUntilConfirmed(1500ms);

  // ⚑ AMENDED 2026-09-06, AGAINST THIS CASE'S OWN AUTHOR.
  //
  // The first version marked the instant the costmap stopped and asserted over
  // EVERY sample after it. That demanded the verdict flip INSTANTLY -- which
  // contradicts the entire reason a budget exists, and would have made the case
  // fail against a CORRECT implementation on the first sample or two, while the
  // age is still inside any sane budget and `watching: yes` is the right answer.
  //
  // A build seat measured exactly that and concluded the case was UNSATISFIABLE.
  // ⚑ IT IS NOT, AND THE CORRECTION RUNS AGAINST BOTH OF US. The defect was in
  // the assertion WINDOW, not in the property: the property this case is about
  // is CONVERGENCE -- a filter nothing is driving must ARRIVE at not-watching --
  // never instantaneity. So the window now opens AFTER any plausible budget has
  // elapsed, and the case asks the question it always meant to ask.
  //
  // ⚑ AND IT MUST STILL BE RED ON CURRENT SOURCE, or the amendment has destroyed
  // the oracle rather than fixed it. With `costmap_silence_timeout <= 0`,
  // `costmap_silent_` can never become true at ANY age, so the converged window
  // still reports `watching: yes` and this case still fails. Verified by running
  // it, not by reasoning about it.
  std::this_thread::sleep_for(1200ms);   // costmap stopped; let a budget elapse
  const size_t m = recorder_->mark();    // ⚑ mark AFTER convergence, not before
  std::this_thread::sleep_for(600ms);
  const auto samples = recorder_->since(m);

  ASSERT_FALSE(samples.empty())
    << "the heartbeat must still be publishing, or this case proves nothing";

  for (const auto & s : samples) {
    EXPECT_NE(get(s, "watching"), "yes")
      << "the costmap is stopped and the filter claims to be watching, because "
         "its detector was configured off. Disabling a detector removes a "
         "measurement; it does not create a good one: " << render(s);
    EXPECT_NE(get(s, "enforced"), "yes")
      << "`enforced: yes` from a stopped costmap -- generation 3 reinstated by "
         "one line of YAML: " << render(s);
    EXPECT_NE(s.level, diagnostic_msgs::msg::DiagnosticStatus::OK)
      << "level OK from a stopped costmap: " << render(s);
  }
}

// ===========================================================================
// SC-2 — NEGATIVE CONTROL FOR SC-1, and it is not optional.
//
// Without it SC-1 is satisfiable by a filter that never says `yes` at all — a
// detector that cannot be wrong because it never asserts anything, which is not
// the property being bought. Same disabled detector; the costmap IS ticking;
// the zone MUST read `yes`.
//
// ⚑ IF SC-2 EVER GOES RED, SC-1 PASSING MEANS NOTHING. Read them as a pair.
// ===========================================================================
TEST_F(SotifGateInertness, SC2_NegativeControl_DisabledDetectorStillReadsYesWhileDriven)
{
  ASSERT_TRUE(build(/*liveness_period=*/0.1, /*silence_timeout=*/0.0));

  driveUntilConfirmed(2000ms);

  const size_t m = recorder_->mark();
  driveUntilConfirmed(600ms);            // keep driving through the window
  const auto samples = recorder_->since(m);

  ASSERT_FALSE(samples.empty());
  bool saw_yes = false;
  for (const auto & s : samples) {
    if (get(s, "enforced") == "yes" && get(s, "watching") == "yes") {saw_yes = true;}
  }
  EXPECT_TRUE(saw_yes)
    << "a driven, confirmed zone never read `yes` -- SC-1 would then be passing "
       "for a reason other than the property under test";
}

// ===========================================================================
// SC-3 — ⚑ THE DECLARED EXPIRY MUST NOT OUTLIVE THE PUBLISHER'S OWN DEFINITION
// OF NOT-WATCHING. THIS IS EXPECTED RED AT SHIPPED DEFAULTS.
//
// Two numbers in this package answer the same consumer question — *how long may
// I keep believing this?* — and they disagree:
//
//   valid_for_s              = liveness_period * 2.5   (cpp:59, :1086)
//                            = 2.5 s at the default liveness_period of 1.0
//   costmap_silence_timeout  = 2.0 s default           (hpp:467)
//
// README:248 tells the consumer: *"If now - header.stamp > valid_for_s, stop
// believing the message."* Read literally — and it is written to be read
// literally, by three lines of code — a consumer keeps believing an
// `enforced: yes` for 2.5 s after the last message. The publisher's own
// threshold for declaring that nobody is watching is 2.0 s. ⚑ THERE IS A 0.5 s
// WINDOW, AT THE SHIPPED DEFAULTS, IN WHICH THE CONSUMER IS ACTING ON A CLAIM
// THE PUBLISHER WOULD ITSELF HAVE DISOWNED.
//
// It widens without bound: neither parameter has a ceiling and nothing relates
// them. `liveness_period: 30.0` yields `valid_for_s: 75.0` — a consumer
// instructed to believe a 75-second-old enforcement claim, which on a moving
// platform is a distance, not a time.
//
// The invariant: an expiry a publisher declares must not exceed the age at
// which that same publisher would call itself stale.
// ===========================================================================
TEST_F(SotifGateInertness, SC3_DeclaredExpiryMustNotExceedTheSilenceBudget)
{
  // Shipped defaults, stated explicitly so the case is about the DEFAULTS and
  // not about a configuration chosen to make it fail.
  ASSERT_TRUE(build(/*liveness_period=*/1.0, /*silence_timeout=*/2.0));

  const size_t m = recorder_->mark();
  driveUntilConfirmed(2500ms);
  const auto samples = recorder_->since(m);
  ASSERT_FALSE(samples.empty());

  for (const auto & s : samples) {
    ASSERT_NE(get(s, "valid_for_s"), "<absent>");
    const double valid_for = std::stod(get(s, "valid_for_s"));
    EXPECT_LE(valid_for, 2.0)
      << "the message declares itself current for " << valid_for
      << "s, while this filter would call its own reading stale at 2.0s. A "
         "consumer honouring the declared expiry acts on a claim the publisher "
         "has already disowned: " << render(s);
  }
}

// ===========================================================================
// SC-4 — `unknown` MUST BE ASSERTED POSITIVELY ON BOTH BRANCHES THAT PRODUCE IT.
//
// MF-6 asserts `enforced == "unknown"` for the STOPPED branch. MF-1 asserts
// only three NEGATIVES for the NEVER-DRIVEN branch — `!= yes`, `watching !=
// yes`, `level != OK` — so a future change emitting `NO` there would pass.
//
// That distinction is load-bearing and the package argues it itself at
// cpp:1003-1010: STALE rather than ERROR, because *"nothing was measured and
// found bad; nothing was measured."* `NO` means measured-and-bad and would send
// an operator to look for a fault that does not exist. A vocabulary whose
// meanings are argued this carefully needs a positive oracle on every branch.
// ===========================================================================
TEST_F(SotifGateInertness, SC4_NeverDrivenReportsUnknownPositivelyNotMerelyNotYes)
{
  ASSERT_TRUE(build(/*liveness_period=*/0.1, /*silence_timeout=*/0.3));

  // Deliberately never tick: the configure->activate gap.
  const size_t m = recorder_->mark();
  std::this_thread::sleep_for(600ms);
  const auto samples = recorder_->since(m);

  ASSERT_FALSE(samples.empty())
    << "the heartbeat must be publishing, or this case proves nothing";

  for (const auto & s : samples) {
    EXPECT_EQ(get(s, "enforced"), "unknown")
      << "the never-driven branch must report `unknown` -- nothing measured -- "
         "and not `NO`, which claims a fault was found: " << render(s);
    EXPECT_EQ(s.level, diagnostic_msgs::msg::DiagnosticStatus::STALE)
      << "never-driven is STALE, not ERROR: " << render(s);
  }
}

// ===========================================================================
// SC-5 — ⚑ THE README'S OWN RECOMMENDED COUNTERMEASURE MUST ACTUALLY MATCH THIS
// PUBLISHER. THIS CASE EXISTS TO REFUTE ITS AUTHOR.
//
// `README.md:450-454` tells an integrator that the complete answer to a dead
// publisher lives in their process, and names it: *"a `DEADLINE` QoS on your
// subscription."* FSE's PI-4 says that advice cannot work here, because
// `decision_pub_` is created with a bare `rclcpp::QoS(10)`
// (zone_parameter_filter.cpp:147), leaving the OFFERED deadline at the rmw
// default of infinity — and DEADLINE is a Request/Offered QoS, so an offered
// infinity does not satisfy any finite request. The subscription would not
// match and the consumer would receive NOTHING.
//
// ⚑ THAT WAS THE ONE FINDING IN THE SOTIF TABLE DERIVED FROM A SPECIFICATION
// RATHER THAN FROM SOURCE READ ON DISK, and it was published as
// CONFIRMED-BY-SPEC / UNVERIFIED-BY-EXECUTION. This case executes it.
//
// TWO ARMS, and the control is not optional: a default-QoS subscriber must
// receive, or a silent deadline arm proves nothing but a broken probe.
//
// ⚑ RESULT DIRECTION. The assertion is written as the property an integrator
// SHOULD get — the recommended subscription receives messages. GREEN means
// PI-4 IS REFUTED and the README advice is sound. RED means the package
// recommends a mechanism it prevents.
//
// ⚑ AND THE ANSWER IS RMW-DEPENDENT. Whatever this returns is true of the rmw
// in this image and does NOT generalise to an integrator's stack. That bound
// rides the result; it is the class of claim this unit has published without
// earning before.
// ===========================================================================
TEST_F(SotifGateInertness, SC5_DeadlineQoSSubscriberMustMatchThisPublisher)
{
  ASSERT_TRUE(build(/*liveness_period=*/0.1, /*silence_timeout=*/2.0));

  auto probe = std::make_shared<rclcpp::Node>("sotif_deadline_probe");
  std::atomic_int default_count{0};
  std::atomic_int deadline_count{0};
  std::atomic_bool incompatible{false};

  auto sub_control = probe->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "zone_decision", rclcpp::QoS(10),
    [&default_count](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr) {
      default_count.fetch_add(1);
    });

  rclcpp::QoS deadline_qos(10);
  deadline_qos.deadline(rclcpp::Duration::from_seconds(2.5));   // = valid_for_s at 1 Hz
  rclcpp::SubscriptionOptions so;
  so.event_callbacks.incompatible_qos_callback =
    [&incompatible](rclcpp::QOSRequestedIncompatibleQoSInfo &) {
      incompatible.store(true);
    };
  auto sub_deadline = probe->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
    "zone_decision", deadline_qos,
    [&deadline_count](diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr) {
      deadline_count.fetch_add(1);
    },
    so);

  exec_->add_node(probe);
  driveUntilConfirmed(2000ms);
  exec_->remove_node(probe);

  ASSERT_GT(default_count.load(), 0)
    << "the DEFAULT-QoS control arm received nothing -- the probe is broken and "
       "the deadline arm's silence would prove nothing";

  EXPECT_GT(deadline_count.load(), 0)
    << "a subscriber requesting a finite DEADLINE -- the mechanism README.md:451 "
       "tells an integrator to use -- received NOTHING from this publisher, which "
       "offers no deadline (QoS(10) at zone_parameter_filter.cpp:147). "
       "incompatible_qos event fired: " << (incompatible.load() ? "YES" : "no")
    << ". control arm received " << default_count.load() << " messages.";
}

// ===========================================================================
// SC-6 — ⚑ A TRIPWIRE UNDER AoU-S1, NOT A DEFECT TEST. IT IS GREEN TODAY AND
// MUST GO RED WHEN THE WORLD CHANGES.
//
// AoU-S1 tells an integrator: this component reports, it cannot stop anything,
// SO THE GATING CODE IS YOURS. That assumption is the most consequential of the
// six, because an integrator who misreads it builds no gate at all.
//
// Its factual basis: CostmapFilter::updateCosts() runs setCurrent(true)
// unconditionally AFTER process() returns (costmap_filter.cpp:133) and is
// declared `final` (costmap_filter.hpp:114). So `Layer::isCurrent()` -- the
// channel nav2 itself reads, via LayeredCostmap::isCurrent() and
// ControllerServer::waitForCostmap() -- cannot carry this filter's fault.
//
// ⚑ BUT THE HEADER ALSO RECORDS A PATCH THAT WOULD CHANGE THIS: moving
// setCurrent(true) AHEAD of process() (hpp:540-547). If that ever lands
// upstream, AoU-S1's basis is gone and the assumption must be rewritten -- and
// nothing today would notice.
//
// So this asserts the CURRENT truth. It passing is the assumption holding.
// ⚑ IT GOING RED IS NOT A REGRESSION -- IT IS THE SIGNAL TO REWRITE AoU-S1.
// That sentence is the whole reason the case exists (Vision 9: the machine
// catches it, not the reader).
// ===========================================================================
TEST_F(SotifGateInertness, SC6_Tripwire_TheFilterCannotTellTheStackItIsNotCurrent)
{
  ASSERT_TRUE(build(/*liveness_period=*/0.1, /*silence_timeout=*/0.3));

  driveUntilConfirmed(800ms);

  // Stop driving. The filter will shortly report NOT WATCHING on its own surface.
  std::this_thread::sleep_for(900ms);

  const auto samples = recorder_->since(0);
  ASSERT_FALSE(samples.empty());
  const auto & last = samples.back();
  ASSERT_EQ(get(last, "watching"), "NO")
    << "precondition: the filter must be reporting NOT WATCHING on its own "
       "surface, or this case is not asking its question: " << render(last);

  EXPECT_TRUE(filter_->isCurrent())
    << "⚑ AoU-S1'S BASIS HAS CHANGED. This filter now CAN carry its fault on "
       "Layer::isCurrent(), the channel nav2 reads. That is good news and it "
       "makes AoU-S1 WRONG -- rewrite it, and tell integrators the stack can "
       "see this after all. Do not 'fix' this test.";
}

// ===========================================================================
// SC-7 — ⚑ DOES THE NAMESPACE JOIN ACTUALLY *DELIVER*, OR IS IT ONLY
// NON-BREAKING? THIS IS THE ONLY CASE IN THE PACKAGE THAT CAN TELL.
//
// `src/zone_parameter_filter.cpp:292` applies `joinWithParentNamespace()` to the
// INTEGRATOR's `node:` field -- the loom we had applied to all four of our own
// topic names and to none of theirs. It closes the phantom-client defect the
// live stack found.
//
// ⚑ BUT EVERY OTHER TEST IN THIS PACKAGE RUNS AT ROOT NAMESPACE, WHERE A JOINED
// AND AN UNJOINED NAME RESOLVE IDENTICALLY. Measured 2026-09-06: `test/` and
// `hubot_live_stack/src/` contain ZERO ROS namespace configuration -- every
// occurrence of the word is a C++ `namespace` block. So the fix was verified as
// NON-BREAKING and never verified as DELIVERING.
//
// ⚑ THAT DISTINCTION IS THE POINT. A loom that looks shared and is not
// delivering is worse than one openly withheld: it removes the visible gap and
// leaves the integrator exactly where they were. It is this package's own
// PI-CLASS -- a success-shaped value -- moved to the delivery layer.
//
// So: host node in `/probe_ns`, target node in `/probe_ns`, and a RELATIVE
// `node: sotif_target_node` in the configuration. If the join delivers, the set
// reaches a real node and `enforced` reaches `yes`. GREEN here is the fix
// DELIVERING, not merely not breaking.
// ===========================================================================
TEST_F(SotifGateInertness, SC7_TheNamespaceJoinDeliversAndNotMerelyDoesNoHarm)
{
  ASSERT_TRUE(
    build(
      /*liveness_period=*/0.1, /*silence_timeout=*/2.0, kZoneCell,
      /*host_ns=*/"robot1/local_costmap", /*target_ns=*/"robot1",
      /*topic_ns=*/"robot1"));

  driveUntilConfirmed(2500ms);

  const auto samples = recorder_->since(0);
  ASSERT_FALSE(samples.empty()) << "nothing published -- the case proves nothing";

  bool delivered = false;
  for (const auto & s : samples) {
    if (get(s, "enforced") == "yes") {delivered = true;}
  }
  EXPECT_TRUE(delivered)
    << "under a NAMESPACED launch, a relative `node:` never reached its target -- "
       "the join at zone_parameter_filter.cpp:292 does not deliver. last sample: "
    << render(samples.back());
}

// ===========================================================================
// SC-8 — NEGATIVE CONTROL FOR SC-7, and without it SC-7 proves nothing.
//
// Same namespaced host, but the target node is left at ROOT. A join that works
// must now MISS it -- the client addresses `/probe_ns/sotif_target_node` and
// nothing serves that. If this arm also reached `yes`, SC-7's green would mean
// only that the name resolves somewhere, not that the join is what carried it.
//
// ⚑ Expected: `NO` after the set_parameters deadline.
// ===========================================================================
TEST_F(SotifGateInertness, SC8_NegativeControl_ARootTargetIsNotReachedFromANamespacedHost)
{
  ASSERT_TRUE(
    build(
      /*liveness_period=*/0.1, /*silence_timeout=*/2.0, kZoneCell,
      /*host_ns=*/"robot1/local_costmap", /*target_ns=*/"",
      /*topic_ns=*/"robot1"));

  driveUntilConfirmed(2500ms);

  const auto samples = recorder_->since(0);
  ASSERT_FALSE(samples.empty());

  const auto & last = samples.back();
  EXPECT_NE(get(last, "enforced"), "yes")
    << "a target sitting at ROOT was reported as enforced from a host in "
       "/probe_ns -- the join is not addressing what it claims to, and SC-7's "
       "green would then prove nothing: " << render(last);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
