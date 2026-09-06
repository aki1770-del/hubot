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
// ⚑ THIS FILE HAS NEVER BEEN COMPILED OR RUN. There is no ROS on the machine it
// was written on (`/opt/ros` does not exist) and no released `nav2_costmap_2d`
// carries `ZONE_PARAMETER_FILTER`, so the whole package cannot build today —
// which is the package's own headline disclosure. Every other suite here
// records a RED-before / GREEN-after; THIS ONE DOES NOT AND MUST NOT CLAIM ONE.
// It is UNVERIFIED, not cleared. The first integrator who can build this
// package should expect SC-1 to FAIL on current source; that expectation is a
// prediction from reading, not a measurement.
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
  InfoPublisher()
  : Node("sotif_info_pub")
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
  : Node("sotif_mask_pub")
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
  TargetNode()
  : Node("sotif_target_node")
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

  DecisionRecorder()
  : Node("sotif_decision_recorder")
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
  bool build(double liveness_period, double silence_timeout, int8_t mask_fill = kZoneCell)
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

    target_node_ = std::make_shared<TargetNode>();
    node_ = std::make_shared<nav2::LifecycleNode>("sotif_host", opts);
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

  const size_t m = recorder_->mark();
  std::this_thread::sleep_for(1500ms);   // the costmap is now stopped
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

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
