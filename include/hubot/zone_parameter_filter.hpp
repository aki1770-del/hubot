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
#include "nav2_msgs/msg/costmap_filter_info.hpp"

namespace hubot
{

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
  /// — no new interface package, and every existing operator tool renders it.
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    decision_pub_;
  std::string decision_topic_{"zone_decision"};

  /// Emit the current zone situation in terms a person can act on.
  void publishDecision(const std::string & detail);

  /// The three-valued enforcement token: "NO", "pending" or "yes".
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

  std::string state_event_topic_;

  bool filter_info_received_{false};
};

}  // namespace hubot

// ============================================================================
// ⚑ ASSUMPTIONS OF USE — read these before relying on this filter.
// Written 2026-09-05 by CPP against nav2 read at /home/komada/nav2ci/ws/src.
// Every line here is a measurement, with the file position that shows it.
//
// AoU-1  THIS FILTER CANNOT TELL THE NAVIGATION STACK THAT ITS OUTPUT IS
//        UNTRUSTWORTHY. The stack's own channel for that is Layer::isCurrent()
//        -- LayeredCostmap::isCurrent() iterates filters_ (layered_costmap.cpp),
//        Costmap2DROS::isCurrent() forwards it (costmap_2d_ros.hpp:186), and
//        ControllerServer::waitForCostmap() (controller_server.cpp:668) blocks
//        on it for `costmap_update_timeout` (default 0.30 s,
//        parameter_handler.cpp:47) before terminating the goal with
//        CONTROLLER_TIMED_OUT (controller_server.cpp:676, caught :634).
//        That channel is closed to this class three ways:
//          (a) CostmapFilter::updateCosts() runs `setCurrent(true)`
//              UNCONDITIONALLY after process() returns (costmap_filter.cpp:133),
//              so any setCurrent(false) inside process() is erased immediately;
//          (b) CostmapFilter::updateCosts() is declared `final`
//              (costmap_filter.hpp:114) -- g++ refuses the override:
//              "error: virtual function ... overriding final function";
//          (c) Layer::isCurrent() is not virtual (layer.hpp:138) and
//              LayeredCostmap holds layers as shared_ptr<Layer>
//              (layered_costmap.hpp:232-233), so it cannot be intercepted.
//        A three-line upstream change would open it (a virtual
//        `isFilterCurrent()` defaulting to true, used as the argument to
//        setCurrent). Until that lands, AN INTEGRATOR MUST SUPPLY THIS
//        THEMSELVES: subscribe to `zone_decision` and refuse to drive on
//        `enforced: NO` or `enforced: pending`. The filter reports; it cannot
//        stop anything.
//
// AoU-2  `enforced: yes` MEANS EVERY TARGET OF THE CURRENT STATE CONFIRMED.
//        `pending` means requested and unanswered. `NO` means at least one
//        target rejected, threw, or fell silent past the deadline. `pending`
//        is not a weaker `yes`; treat it as `NO` until it resolves.
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
// AoU-5  THE FILTER ONLY LEARNS WHAT THE TARGETS SAID WHEN THE COSTMAP TICKS.
//        checkPendingParameterUpdates() runs from the top of process() and
//        nowhere else, because a costmap filter owns no timer. If costmap
//        updates stop, results are never examined, `pending` never resolves,
//        and the deadline in AoU-2 never fires -- the last published value
//        simply stands. Do not read a stale DiagnosticArray as a live one; it
//        carries a header stamp, so check it.
// ============================================================================

#endif  // HUBOT__ZONE_PARAMETER_FILTER_HPP_
