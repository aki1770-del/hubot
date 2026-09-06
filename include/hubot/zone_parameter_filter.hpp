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

#ifndef HUBOT__ZONE_PARAMETER_FILTER_HPP_
#define HUBOT__ZONE_PARAMETER_FILTER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "nav2_costmap_2d/costmap_filters/costmap_filter.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace hubot
{

/// ⚑ THE `CostmapFilterInfo.type` DISCRIMINATOR THIS FILTER ANSWERS TO -- ours,
/// declared here, since 2026-09-06.
///
/// It was `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, which exists on nav2
/// branches `main` and `lyrical` and in NO release tag -- so for a day this
/// package "could not build against a released nav2". Measured 2026-09-06: that
/// symbol was the ONLY blocker, at eight sites; with it substituted, hubot
/// builds and installs against release tag 1.5.1 with the whole nav2 dependency
/// chain built from that tag.
///
/// We never needed to OBTAIN the number from nav2's header. It is a wire value:
/// the integrator's own `costmap_filter_info_server` publishes it out of their
/// YAML and this filter only has to AGREE with it. Importing the symbol coupled
/// our buildability to an upstream header revision for no benefit.
///
/// It is 4 because upstream numbers KEEPOUT_FILTER=0, SPEED_FILTER_PERCENT=1,
/// SPEED_FILTER_ABSOLUTE=2, BINARY_FILTER=3, and the zone-parameter filter is
/// the next one. Two static_asserts keep that honest: the first fires on ANY
/// nav2 if upstream renumbers the existing filters; the second fires where
/// upstream carries ZONE_PARAMETER_FILTER and disagrees with us. A bare literal
/// with no cross-check -- the Phase A probe -- would have traded a loud build
/// failure for a quiet runtime one, which is the reassuring-value family one
/// layer out, and it must not ship.
inline constexpr uint8_t kZoneParameterFilterType = 4;

// Always on: these constants exist in EVERY nav2. Ours sits directly after
// BINARY_FILTER; if upstream ever renumbers, this fires on release and branch.
static_assert(
  kZoneParameterFilterType == nav2_costmap_2d::BINARY_FILTER + 1,
  "hubot::kZoneParameterFilterType must be BINARY_FILTER + 1: upstream nav2 has "
  "renumbered its costmap-filter discriminators and this filter would answer to "
  "the wrong CostmapFilterInfo.type on the wire");

#if defined(HUBOT_UPSTREAM_HAS_ZONE_PARAMETER_FILTER)
// Defined by CMakeLists.txt when a compile probe finds the SYMBOL upstream (a
// header probe cannot: filter_values.hpp exists on both substrates). Where nav2
// carries the constant, a disagreement is a COMPILE error, never a silent wire
// mismatch.
static_assert(
  kZoneParameterFilterType == nav2_costmap_2d::ZONE_PARAMETER_FILTER,
  "hubot::kZoneParameterFilterType disagrees with nav2_costmap_2d::ZONE_PARAMETER_FILTER "
  "on this nav2; the two must name the same CostmapFilterInfo.type value");
#endif

/**
 * @class ZoneParameterFilter
 * @brief Costmap filter that applies a configured set of ROS parameters
 *        based on the mask value at the robot's pose.
 *
 * The filter mask uses occupancy-grid values. State 0 is the reset state;
 * each non-zero state ID maps via configuration to a list of parameter
 * overrides on configured target nodes.
 */
class ZoneParameterFilter : public nav2_costmap_2d::CostmapFilter
{
public:
  ZoneParameterFilter();

  /**
   * @brief Initialise filter, subscribe to filter info / mask, build
   *        per-target-node async parameter clients.
   */
  void initializeFilter(const std::string & filter_info_topic) override;

  /**
   * @brief Sample the mask at the robot pose; if the state changed,
   *        apply the new state's parameter set via async client and,
   *        if configured, publish the state event.
   */
  void process(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j,
    const geometry_msgs::msg::Pose & pose) override;

  /**
   * @brief Reset filter — drop subscriptions, reset publisher, and ACTUALLY
   *        drop the loaded configuration.
   *
   * ⚑ Until 2026-09-05 it dropped none of the configuration: `state_param_map_`,
   * `nominal_defaults_` and `param_clients_` had no `.clear()` anywhere in the
   * translation unit. CostmapFilter::reset() (costmap_filter.cpp:103) calls
   * resetFilter() and then initializeFilter(), which re-runs loadStateConfig()
   * over those same containers -- and nominal_defaults_ is filled with
   * push_back. So every ClearEntireCostmap recovery DUPLICATED every nominal
   * default, without bound, and left clients for targets the new configuration
   * had removed.
   *
   * It does NOT clear `degraded_targets_`; see `enforcement_degraded_`.
   */
  void resetFilter() override;

  /**
   * @brief Whether the filter has received its mask and is operational.
   */
  bool isActive();

  /// ⚑ HUBOT: true once a parameter set has failed, i.e. the zone is known NOT to
  /// be enforced on at least one target. Public on purpose — upstream expressed
  /// this condition by aborting the process, which told an integrator nothing they
  /// could act on. An integrator can read this and decide.
  ///
  /// ⚑ HONEST BOUND, measured 2026-09-05: NOTHING IN NAV2 CALLS THIS. It is
  /// reachable only by an integrator holding the concrete type, which the
  /// pluginlib load path does not give them. The channel nav2 itself reads is
  /// `Layer::isCurrent()`, and this filter CANNOT drive it: CostmapFilter
  /// sets `current_ = true` unconditionally after process() returns
  /// (costmap_filter.cpp:133) and declares updateCosts() `final`
  /// (costmap_filter.hpp:114), so the correction cannot be applied from a
  /// derived class -- g++ rejects the override outright. Read the
  /// `zone_decision` DiagnosticArray, not this accessor, unless you own the
  /// pointer. See ASSUMPTIONS OF USE at the foot of this file.
  bool enforcementDegraded() const {return enforcement_degraded_;}

  /// True while the mask names a state the configuration never declared. The
  /// robot is somewhere the YAML does not describe, so no zone limits were
  /// applied for it and the values in force are the previous zone's.
  bool maskStateUndeclared() const {return mask_state_undeclared_;}

  /// ⚑ HUBOT: false while the costmap is still calling process(), true once it
  /// has stopped for longer than `costmap_silence_timeout`. Read `watching` on
  /// the `zone_decision` DiagnosticArray rather than this, for the same reason
  /// as `enforcementDegraded()` -- pluginlib does not hand you the concrete
  /// type.
  bool costmapSilent() const {return costmap_silent_;}

protected:
  /**
   * @brief Subscriber callback for the filter info topic.
   */
  void filterInfoCallback(
    const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg);

  /**
   * @brief Subscriber callback for the filter mask topic.
   */
  void maskCallback(
    const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg);

  /**
   * @brief Parse the per-state parameter map and nominal_defaults from
   *        YAML overrides.
   */
  void loadStateConfig();

  /**
   * @brief Apply the parameter set associated with the given state.
   *        State 0 restores nominal_defaults.
   * @return true if the state was applied (its sets were ISSUED -- not
   *         confirmed); false if the mask named a state the configuration
   *         never declared, in which case NOTHING was issued and the caller
   *         MUST NOT record `new_state` as the current state.
   *
   * ⚑ The bool is the whole point. This used to be void and to THROW on an
   * undeclared state, out of process(), into CostmapFilter::updateCosts(),
   * which is bare -- so a mask-authoring typo killed the navigation node.
   * Returning false instead is only half a fix: a caller that degrades but
   * still records the undeclared value leaves the previous zone's overrides
   * applied while believing it is somewhere else, and the next transition's
   * N-only reset then misses. See process().
   */
  bool applyState(uint8_t new_state);

  /**
   * @brief Restore all overridden parameters to their nominal_defaults
   *        values via async set_parameters.
   */
  void resetToNominal();

  /**
   * @brief Issue an async set_parameters call to the named target node.
   */
  void issueAsyncSetParameters(
    const std::string & target_node,
    const std::vector<rclcpp::Parameter> & params);

  /**
   * @brief Record that a target is known NOT to be carrying the zone's values,
   *        and keep `enforcement_degraded_` in step. Idempotent.
   */
  void markTargetDegraded(const std::string & target_node, const std::string & why);

  /**
   * @brief Record that a target answered successfully, which is the ONLY event
   *        that retracts a fault against it, and keep the flag in step.
   */
  void markTargetHealthy(const std::string & target_node);

  /**
   * @brief Process completed set_parameters results non-blockingly. Called
   *        at the start of every process().
   *
   *        A failed set marks that TARGET degraded and the filter keeps
   *        running; it does NOT throw. It used to, and this sentence used to
   *        say so after the code had stopped doing it -- which made the
   *        integrator's primary interface document state the exact policy
   *        this package exists to reverse, and it is the sentence they would
   *        read to decide they need no fallback of their own.
   *
   *        A set that never answers is also a failure, on a deadline
   *        (`set_parameters_timeout`). Silence is the likeliest way a zone
   *        goes unenforced -- the target crashed, was never brought up, is
   *        still configuring, or the namespace in the YAML has a typo -- and
   *        it is the one failure that otherwise looks exactly like success.
   */
  void checkPendingParameterUpdates();

  /**
   * @brief ⚑ HUBOT — THE LIVENESS TICK. The only thing in this class that runs
   *        WITHOUT the costmap.
   *
   *        It does two jobs, and the second is the one the package is named
   *        for:
   *
   *        (1) it runs checkPendingParameterUpdates(), which needs the clock
   *            and the futures and NOTHING from the costmap -- so deadlines
   *            fire and `pending` resolves even when nothing is ticking;
   *        (2) it PUBLISHES, every period, whether or not anything changed, so
   *            that the presence of the message is itself the statement "I am
   *            watching" and the ABSENCE of it is the statement "I am not".
   *
   *        (2) is not optional. Publishing only on change makes a healthy quiet
   *            filter and a dead one produce the identical observable, and an
   *            operator cannot act on an observable that is the same in both.
   */
  void livenessTick();

  /// Seconds since process() last ran, on the STEADY clock. Steady, not ROS
  /// time, on purpose: the question is whether a THREAD is still turning, which
  /// is a wall-clock fact. A stalled `/clock` would freeze a ROS-time age at
  /// 0.000 and report perfect health for a stopped stack -- absence riding the
  /// measurement scale, which is the shape of defect this whole surface exists
  /// to refuse.
  double secondsSinceLastProcess() const;

  nav2::Subscription<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr filter_info_sub_;
  nav2::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;
  nav2::Publisher<std_msgs::msg::UInt8>::SharedPtr state_event_pub_;

  nav_msgs::msg::OccupancyGrid::ConstSharedPtr filter_mask_;
  std::string global_frame_;

  uint8_t current_state_{0};
  bool state_initialized_{false};

  /// ⚑ HUBOT: true while at least one target is known NOT to be carrying the
  /// zone's values. Kept in step with `degraded_targets_`; it is a cache of
  /// `!degraded_targets_.empty()`, never set on its own.
  ///
  /// Atomic because it is read outside the filter mutex -- by the public
  /// accessor, and by publishDecision(). `Layer::current_` next door is
  /// `std::atomic_bool` for the same reason (layer.hpp:197).
  ///
  /// ⚑ It is NOT cleared when the robot leaves the mask. Leaving the mask only
  /// ISSUES the restore; nothing has confirmed it yet, and reporting
  /// `enforced: yes` on an unconfirmed restore is the success-shaped value this
  /// whole feature exists to abolish.
  ///
  /// ⚑ AND IT IS NOT CLEARED BY resetFilter() EITHER -- it was, until
  /// 2026-09-05, on the stated ground that "the configuration it referred to is
  /// gone". That ground was false twice over. First, this flag is a claim about
  /// a TARGET NODE'S LIVE PARAMETER VALUE; reloading a costmap filter does not
  /// reach the target, so nothing about the reload refutes the claim. Second,
  /// resetFilter() is not a rare event: ClearEntireCostmap appears in seven of
  /// nav2's default behaviour trees, reaching Costmap2DROS::resetLayers()
  /// (costmap_2d_ros.cpp:719) and CostmapFilter::reset() (costmap_filter.cpp:103),
  /// so an ORDINARY RECOVERY erased the fault flag while the fault stood.
  ///
  /// The only thing that clears it is the only thing that can: a set to that
  /// target that actually COMES BACK SUCCESSFUL. You retract a fault by fixing
  /// it, not by forgetting it.
  std::atomic_bool enforcement_degraded_{false};

  /// Targets with a set that failed, was rejected, or never answered. The
  /// authority behind `enforcement_degraded_`. A target leaves this set on one
  /// event only: a successful set_parameters result for it.
  std::set<std::string> degraded_targets_;

  /// Targets with a set ISSUED for the current state and not yet answered.
  /// Non-empty means "requested, unconfirmed" -- which is neither `yes` nor
  /// `NO`, and publishing it as `yes` was the same success-shaped value the
  /// leave-the-mask path above already refuses to publish. The code applied its
  /// own principle on the way out of a zone and violated it on the way in.
  std::set<std::string> unconfirmed_targets_;

  /// ⚑ HUBOT: the state the MASK reports at the robot's pose, which is NOT
  /// always the state whose parameters are in force. They diverge exactly when
  /// the mask names a state the configuration never declared. Publishing both
  /// is the point: a person can see "the mask says zone 7; zone 3's limits are
  /// what is actually applied" and decide. One number cannot carry that.
  uint8_t mask_state_{0};

  /// True while the last sampled mask cell named an undeclared state. Unlike
  /// `enforcement_degraded_` this is NOT latched -- it describes where the
  /// robot is now, and it clears the moment the robot reaches a declared zone.
  std::atomic_bool mask_state_undeclared_{false};

  /// ⚑ HUBOT — THE HUMAN-DECISION SURFACE.
  /// The upstream filter publishes a bare `std_msgs/UInt8`: a state number. A
  /// number is what a ROBOT needs — it selects a parameter set. A HUMAN deciding
  /// whether to trust the zone needs the BASIS: which zone, what it changed, on
  /// which targets, and above all WHETHER IT ACTUALLY TOOK EFFECT.
  /// `diagnostic_msgs/DiagnosticArray` is the ROS-native surface for exactly that
  /// -- no new interface package needed.
  ///
  /// ⚑ IT USED TO SAY "and every existing operator tool renders it." That asserted an
  /// outcome in software we have never run. It is also wrong in a way that matters:
  /// we publish on `zone_decision` joined to the costmap's namespace, NOT on
  /// `/diagnostics`, which is what `diagnostic_aggregator` and `rqt_robot_monitor`
  /// subscribe to by convention -- so the convention tools do NOT find this topic
  /// without a remap. The TYPE is shared; the TOPIC is not. Every subscriber that
  /// exists today is one of this package's own tests or its harness.
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    decision_pub_;
  std::string decision_topic_{"zone_decision"};

  /// Emit the current zone situation in terms a person can act on.
  void publishDecision(const std::string & detail);

  /// ⚑ THE ONE PLACE THE "IS ANYBODY LOOKING" QUESTION IS ANSWERED, and it is a
  /// function rather than a flag so that a new caller cannot fail to ask it.
  ///
  /// This is the FOURTH generation of one defect family in this package, and
  /// the first three were all fixed at a CALLER:
  ///   1. `enforced: yes` on the cycle the sets were ISSUED (nothing confirmed);
  ///   2. `enforced: yes` at STARTUP, having set nothing and read no mask;
  ///   3. `enforced: yes` during SILENCE, because the thing that would have
  ///      refuted it had stopped running -- fixed by adding `unknown`;
  ///   4. `enforced: yes` BEFORE THE FIRST TICK -- because `livenessTick()` was
  ///      a new caller, and it arrived without the guard the third fix had put
  ///      in `publishDecisionOnStatusChange()`.
  ///
  /// Every fix went into a caller and every new caller arrived without it. So
  /// this one goes into the VALUE. `enforcementToken()` and the `watching`
  /// field both derive from here, and neither can be computed around it.
  ///
  /// `ever_processed_` is the discriminator, and it already existed: it was
  /// written in three places and read in exactly one -- inside a message
  /// string, never in the level or the token. It is consulted by the value now.
  ///
  /// NOT-YET-DRIVEN and STOPPED are deliberately the same answer, because to a
  /// consumer they are the same fact: nobody is watching. `initializeFilter()`
  /// runs at CONFIGURE and starts the heartbeat; `map_update_thread_` is not
  /// created until ACTIVATE (costmap_2d_ros.cpp:314). The gap between them is
  /// every ordinary bringup, not an edge case -- and `resetFilter()` re-opens
  /// it on every ClearEntireCostmap.
  /// ⚑ DELIBERATELY TWO DISJUNCTS, NOT THREE -- 2026-09-06, and the third one
  /// was tried and MEASURED WRONG rather than reasoned away.
  ///
  /// `test/sotif_gate_inertness_test.cpp` SC-1 proposes, in its own comment,
  /// `|| costmap_silence_timeout_ <= 0.0` as the fix. It was implemented
  /// exactly as written and it turns SC-1 GREEN and SC-2 **RED** -- and SC-2 is
  /// that file's own non-optional negative control, which states: *if SC-2 ever
  /// goes red, SC-1 passing means nothing.* A constant disjunct cannot tell a
  /// driven costmap from a stopped one, so it buys SC-1 by making the filter
  /// never assert `yes` at all, which is the degenerate pass SC-2 exists to
  /// reject.
  ///
  /// The gate here was always right; its INPUT was the thing that could not
  /// move. The fallback silence budget now lives in livenessTick(), so
  /// `costmap_silent_` becomes reachable with the detector disabled and this
  /// function is left alone. Both cases pass on the input fix.
  bool notWatching() const {return !ever_processed_ || costmap_silent_;}

  /// How long a `zone_decision` message declares itself current, AND the
  /// DEADLINE this publisher offers -- ONE number computed in one place, so the
  /// promise in the payload and the promise on the QoS channel cannot drift.
  /// `liveness_period x kValidForPeriods`, CLAMPED to `costmap_silence_timeout`
  /// while that detector is on: a declared expiry must never exceed the age at
  /// which this filter would call its own reading stale (SC-3). With the
  /// detector off, the fallback silence budget IS `liveness_period x
  /// kValidForPeriods`, so the two are equal by construction. 0 when the
  /// heartbeat is disabled, because nothing periodic is then promised.
  double declaredValidForS() const;

  /// The four-valued enforcement token: "NO", "unknown", "pending" or "yes".
  /// `unknown` is not a weaker `yes`; a consumer must treat it as `NO`. The
  /// documented consumer rule is a WHITELIST -- act only on "yes" -- because a
  /// blacklist fails open the moment a fifth value is added.
  std::string enforcementToken() const;

  /// Publish only when that token has CHANGED since the last publish.
  ///
  /// ⚑ Added 2026-09-05 because CPP_N3 caught the bug it fixes. Confirmation
  /// arrives inside checkPendingParameterUpdates(), which had nothing to say
  /// when the news was GOOD -- so a healthy filter published `pending` once and
  /// then went silent, and an integrator watching `zone_decision` could never
  /// tell a confirmed zone from one still waiting. That is the same defect as
  /// N-3 wearing the other face: N-3 published a reassuring value too early,
  /// this published an alarming one forever. Both train an operator to stop
  /// reading the field.
  void publishDecisionOnStatusChange();

  /// Last token actually put on the wire, so the above can tell.
  std::string last_published_token_;
  /// The last non-empty `event` reason, re-published on every heartbeat so a
  /// reason outlives the single volatile message that carried it. Written only
  /// from publishDecision(), which its callers enter holding the costmap mutex
  /// -- the same guarantee `last_published_token_` above already relies on.
  std::string last_event_;

  // One per-state-override or per-nominal-default entry.
  struct StateParamEntry
  {
    std::string target_node;
    rclcpp::Parameter param;
  };
  std::map<uint8_t, std::vector<StateParamEntry>> state_param_map_;
  // Keyed by target_node; values are bare-named Parameters to restore.
  std::map<std::string, std::vector<rclcpp::Parameter>> nominal_defaults_;
  std::map<std::string, rclcpp::AsyncParametersClient::SharedPtr> param_clients_;

  // Client is held with its future: destroying it early breaks the future.
  //
  // ⚑ `target_node` and `issued_at` were added 2026-09-05. Without the target
  // name a failure could not say WHICH node was not carrying the zone -- the
  // error message read "on that target" and never named it -- and a fault could
  // never be retracted, because nothing knew what a later success referred to.
  // Without the timestamp there was no failure at all for the likeliest fault:
  // a future that simply never becomes ready.
  //
  // This struct and `pending_sets_` were both declared before this date and
  // referenced ZERO times by the source, which used a bare vector of futures
  // instead. A declared-and-unused member is a claim the class does not keep.
  struct PendingSet
  {
    std::string target_node;
    rclcpp::AsyncParametersClient::SharedPtr client;
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future;
    rclcpp::Time issued_at;
  };
  std::vector<PendingSet> pending_sets_;

  /// Deadline for one set_parameters round-trip, seconds. <= 0 disables the
  /// deadline and restores the pre-2026-09-05 behaviour of waiting forever.
  /// Declared as `<filter>.set_parameters_timeout`.
  double set_parameters_timeout_{5.0};

  // ⚑ Bounds in-flight sets against a target that answers too slowly to keep
  // up with the costmap rate. It does NOT bound "a target that never answers"
  // -- the comment here said it did, and one unanswered set sits at a count of
  // one forever, so a size bound can never fire on it. The deadline above is
  // what bounds silence; this bounds unbounded GROWTH, and is the backstop.
  static constexpr size_t kMaxPendingSets = 64;

  // =========================================================================
  // ⚑ HUBOT — THE LIVENESS SIGNAL.
  //
  // THE DEFECT IT CLOSES, in this file's own previous words (AoU-5, below):
  // "checkPendingParameterUpdates() runs from the top of process() and nowhere
  // else, because a costmap filter owns no timer. If costmap updates stop,
  // results are never examined, `pending` never resolves, and the deadline
  // never fires -- the last published value simply stands." The countermeasure
  // offered there was the four words "so check it", addressed to the reader.
  //
  // ⚑ "A COSTMAP FILTER OWNS NO TIMER" IS TRUE OF THE BASE CLASS AND FALSE AS A
  // CONSTRAINT, and the difference is the whole design. Measured against the
  // nav2 this builds on:
  //
  //   (a) `Layer::node_` is a live handle in the PROTECTED region
  //       (layer.hpp:186, region opens :181), so a derived filter may create
  //       node entities. This class already creates two subscriptions and two
  //       publishers on it. A timer is the same class of object.
  //   (b) THE NODE IS SPUN ON A DIFFERENT THREAD FROM THE COSTMAP UPDATE LOOP.
  //       Costmap2DROS holds `map_update_thread_` running mapUpdateLoop()
  //       (costmap_2d_ros.cpp:314, :533) -- that is the thread that reaches
  //       process() and that is the thread that can stop. Its node is spun
  //       separately by the server that owns it:
  //       `costmap_thread_ = std::make_unique<nav2::NodeThread>(costmap_ros_)`
  //       (controller_server.cpp:72, planner_server.cpp:78), and NodeThread
  //       spins the node's default callback group in a thread of its own
  //       (node_thread.hpp:40-44).
  //   (c) So the two fail INDEPENDENTLY, and a wall timer on the node keeps
  //       running through exactly the failure AoU-5 describes.
  //
  // ⚑ AND IT COVERS A SECOND DOOR AoU-5 NEVER NAMED. CostmapFilter::updateCosts()
  // returns early when `enabled_` is false and never calls process()
  // (costmap_filter.cpp:128-130), and `enabled_` is flipped by the
  // `<name>/toggle_filter` service the base class publishes
  // (costmap_filter.cpp:82-86) and by the `<name>.enabled` parameter (:74). A
  // healthy stack with a disabled filter therefore produced the same frozen
  // `enforced: yes` as a stopped one. The timer is not gated on `enabled_`, so
  // it reports that too.
  //
  // ⚑ WHAT THIS DOES NOT CLOSE, stated here and not only in the README: IF THE
  // WHOLE NODE DIES, THE TIMER DIES WITH IT and the last message stands, exactly
  // as before. Nothing inside a process can announce that process's own death.
  // That is why every message also carries `report_period_s` and `valid_for_s`
  // -- a PROMISE with a number in it, which a three-line consumer can evaluate,
  // rather than the bare header stamp and an instruction to check it. It is a
  // strictly better bound and it is still a bound.
  // =========================================================================

  /// The timer. Created at the END of initializeFilter() (after the config is
  /// loaded, so a tick landing immediately sees a complete filter) and dropped
  /// in resetFilter(), which is the same lifecycle the subscriptions follow.
  rclcpp::TimerBase::SharedPtr liveness_timer_;

  /// Heartbeat period, seconds. <= 0 disables the whole mechanism and restores
  /// the pre-2026-09-06 behaviour of speaking only on change. Declared as
  /// `<filter>.liveness_period`. 1.0 s matches the rate ROS diagnostics are
  /// conventionally published at.
  double liveness_period_{1.0};

  /// How long process() may be absent before the filter declares it is not
  /// watching, seconds. <= 0 disables the staleness verdict while leaving the
  /// heartbeat running. Declared as `<filter>.costmap_silence_timeout`.
  ///
  /// ⚑ Under `use_sim_time` with a sim running slower than real time, a costmap
  /// ticking at its configured SIM rate can exceed a REAL-time timeout. Raise
  /// this, or accept that a slow sim is correctly reported as not watching --
  /// which it is.
  double costmap_silence_timeout_{2.0};

  /// steady_clock nanoseconds at the last process() entry. Atomic because the
  /// timer thread reads what the costmap update thread writes.
  std::atomic<int64_t> last_process_steady_ns_{0};

  /// False until process() has run even once, so the first report can say
  /// "has not ticked yet" rather than "stopped". A filter that is loaded and
  /// never driven is a real failure and it now has a distinct sentence.
  std::atomic_bool ever_processed_{false};

  /// ⚑ The verdict. NOT latched -- unlike `enforcement_degraded_`, this
  /// describes what is happening now, and the costmap resuming retracts it.
  /// A signal that can only ever go one way stops being read.
  std::atomic_bool costmap_silent_{false};

  /// Monotonic publish counter. ⚑ THE ONE FIELD NO STOPPED CLOCK CAN FAKE: a
  /// reader that sees it advance knows something is still executing, whatever
  /// the header stamp says.
  uint64_t report_seq_{0};

  std::string state_event_topic_;

  bool filter_info_received_{false};
};

}  // namespace hubot

// ============================================================================
// ⚑ ASSUMPTIONS OF USE — read these before relying on this filter.
// Written 2026-09-05 by CPP against upstream ros-navigation/navigation2 main
// @ 0e9904bb (2026-09-04), fetched and re-measured 2026-09-05. Every line here
// is a measurement, with the file position that shows it.
//
// AoU-1  THIS FILTER CANNOT *YET* TELL THE NAVIGATION STACK THAT ITS OUTPUT IS
//        UNTRUSTWORTHY -- BUT THE REASON IS ONE LINE, NOT AN ARCHITECTURE.
//        ⚑ CORRECTED 2026-09-05. An earlier version of this note concluded the
//        channel was "closed three ways" and that an integrator MUST supply the
//        capability themselves. The three facts below are all still true and
//        re-verified against main @ 0e9904bb; the CONCLUSION drawn from them was
//        wrong, and it was wrong in the direction that costs an integrator work.
//
//        The stack's channel for "do not plan on this yet" is Layer::isCurrent()
//        -- LayeredCostmap::isCurrent() iterates filters_
//        (layered_costmap.cpp:291, over layered_costmap.hpp:232-233),
//        Costmap2DROS::isCurrent() forwards it (costmap_2d_ros.hpp:186), and
//        ControllerServer::waitForCostmap() (controller_server.cpp:671) blocks on
//        it for `costmap_update_timeout` (default 0.30 s,
//        parameter_handler.cpp:47-48) before throwing ControllerTimedOut
//        (controller_server.cpp:679, caught :637).
//
//        THE FACTS, RE-MEASURED:
//          (a) CostmapFilter::updateCosts() runs `setCurrent(true)`
//              unconditionally AFTER process() returns (costmap_filter.cpp:133),
//              so any setCurrent(false) made inside process() is erased;
//          (b) CostmapFilter::updateCosts() is declared `final`
//              (costmap_filter.hpp:114), so the method cannot be overridden;
//          (c) Layer::isCurrent() is not virtual (layer.hpp:138) and
//              LayeredCostmap holds layers as shared_ptr<Layer>, so the READ
//              cannot be intercepted.
//
//        WHAT FOLLOWS FROM THEM -- AND WHAT DOES NOT. (b) and (c) say the
//        capability cannot be added by SUBCLASSING. They do not say it is
//        absent. Layer::setCurrent(bool) is PUBLIC and non-virtual
//        (layer.hpp:147, inside the `public:` region that opens at :61 and ends
//        at :181), so this class may already call setCurrent(false) from
//        process() today. The write happens; it is simply overwritten one line
//        later by (a). The capability EXISTS in the base class and is ERASED by
//        statement ORDER in the derived one -- and nav2's own header says it
//        should not be: "A layer's current state should be managed by the
//        protected variable current_" (layer.hpp:134-135) and, on the member
//        itself, "Currently this var is managed by subclasses." (layer.hpp:198).
//
//        THE UPSTREAM FIX IS ONE STATEMENT MOVED, not a new virtual. Moving
//        `setCurrent(true)` ahead of `process()` restores the documented
//        contract with a byte-identical header and zero ABI surface. Our patch,
//        with a test proven RED before and GREEN after, is at
//        outputs/cpp/nav2_costmap_filter_setcurrent_before_process_2026_09_05.patch.
//        (An earlier design of ours added a virtual `isFilterCurrent()`; it is
//        RETIRED as superseded -- it changed the vtable to buy what statement
//        order already gives.)
//
//        UNTIL THAT LANDS UPSTREAM, the integrator guidance is unchanged and
//        still required: subscribe to `zone_decision` and
//        **PROCEED ONLY ON `enforced: yes`.** Not because the stack cannot
//        carry the signal -- it can -- but because the nav2 you link against
//        today still erases it. The filter reports; it cannot stop anything.
//
//        ⚑ CORRECTED 2026-09-06. This line read "refuse to drive on
//        `enforced: NO` or `enforced: pending`" until now -- A BLACKLIST, and
//        the package's own CHANGELOG had already declared that exact form
//        BREAKING and named its hazard: a consumer coded literally against it
//        reads the fourth value `unknown` AS PERMISSION, and drives a zone
//        nobody is watching. The whitelist correction reached the README and
//        AoU-2 below and MISSED THIS LINE -- which is the line an integrator
//        reads first, in the header they compile against. Found by DIA, not by
//        the pen that wrote the correction. See AoU-2 for the full vocabulary.
//
// AoU-2  `enforced: yes` MEANS EVERY TARGET OF THE CURRENT STATE CONFIRMED.
//        `pending` means requested and unanswered. `NO` means at least one
//        target rejected, threw, or fell silent past the deadline. `pending`
//        is not a weaker `yes`; treat it as `NO` until it resolves.
//
//        ⚑ AMENDED 2026-09-06 — THERE IS A FOURTH VALUE, `unknown`, AND THE
//        CONSUMER RULE THAT WENT WITH THIS NOTE WAS THE WRONG SHAPE. `unknown`
//        means the costmap has stopped calling process(), so nothing here is a
//        live reading; see `costmap_silent_`. The guidance on the README said
//        "refuse to drive on `enforced: NO` or `pending`" -- A BLACKLIST, which
//        cannot be complete, and which a consumer coded literally against would
//        have read `unknown` as permission. THE RULE IS NOW A WHITELIST:
//        **PROCEED ONLY ON `enforced: yes`. Anything else means the zone's
//        limits are not known to be applied.** Changing a published vocabulary
//        would normally be a breaking act; it is done now, deliberately, while
//        this package has zero remotes and no consumer can be holding the old
//        one -- the cheapest moment it will ever be, and the alternative was to
//        ship a rule that fails open.
//
// AoU-3  A FAULT SURVIVES ClearEntireCostmap ON PURPOSE. See
//        `enforcement_degraded_`. It clears when a set to that target comes
//        back successful, or when a configuration reload stops naming that
//        target at all -- the second is a deliberate operator act, not a
//        recovery behaviour.
//
// AoU-4  THE FILTER NEVER CONFIRMS THE VALUE, ONLY THE ACCEPTANCE. A
//        successful SetParametersResult says the target accepted the set. It
//        does not say the target is still holding it: anything else may set the
//        same parameter afterwards, and this filter will not notice. UNVERIFIED
//        by construction, and out of scope for a costmap filter.
//
// AoU-5  ⚑ REPLACED 2026-09-06. THIS NOTE IS FALSE AS IT STOOD AND THE OLD TEXT
//        IS KEPT BECAUSE IT IS THE REASON THE REPLACEMENT EXISTS.
//
//        IT SAID: "THE FILTER ONLY LEARNS WHAT THE TARGETS SAID WHEN THE
//        COSTMAP TICKS. checkPendingParameterUpdates() runs from the top of
//        process() and nowhere else, because a costmap filter owns no timer. If
//        costmap updates stop, results are never examined, `pending` never
//        resolves, and the deadline in AoU-2 never fires -- the last published
//        value simply stands. Do not read a stale DiagnosticArray as a live
//        one; it carries a header stamp, so check it."
//
//        Every clause of that was true of the code and the last one -- "check
//        it" -- put the detection on the reader. A filter built to abolish
//        values that LOOK like success was itself producing one: an `OK` from
//        ninety seconds ago renders exactly like an `OK` from now.
//
//        WHAT IS TRUE NOW. A liveness timer on the node runs
//        checkPendingParameterUpdates() independently of the costmap, so
//        `pending` DOES resolve and the AoU-2 deadline DOES fire with the
//        costmap stopped; and every report carries `watching`, `costmap_age_s`
//        and a monotonic `report_seq`, published every `liveness_period`
//        whether or not anything changed. Silence on `zone_decision` is
//        therefore no longer the same observable as health. See the block at
//        `liveness_timer_` for the measurements that make the timer possible,
//        and `test/costmap_silence_liveness_test.cpp` for the oracle that
//        separates a STOPPED filter from a RUNNING-AND-QUIET one -- with two
//        negative controls that require this defect to reappear when the
//        mechanism is switched off.
//
//        ⚑ WHAT REMAINS TRUE, AND IS THE HONEST RESIDUE: IF THE NODE ITSELF
//        DIES, THE TIMER DIES WITH IT. No message can then arrive and the last
//        one stands. Nothing running inside a process can announce that
//        process's own death; that is not an omission, it is the shape of the
//        problem. What the messages carry instead is a PROMISE WITH A NUMBER --
//        `report_period_s` and `valid_for_s` -- so a reader (or a three-line
//        consumer) evaluates an expiry the publisher declared, rather than a
//        bare stamp and an instruction. Strictly better than "check it".
//        STILL A BOUND. The only complete answer to a dead publisher lives in
//        the reader's process, not ours: a subscription DEADLINE QoS, or a
//        `diagnostic_aggregator` staleness rule. NOT IMPLEMENTED HERE and NOT
//        MEASURED -- naming it is not the same as having built it.
// ============================================================================

#endif  // HUBOT__ZONE_PARAMETER_FILTER_HPP_
