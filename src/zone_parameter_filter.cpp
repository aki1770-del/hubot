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

#include "hubot/zone_parameter_filter.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"
#include "nav2_util/occ_grid_utils.hpp"

namespace hubot
{

ZoneParameterFilter::ZoneParameterFilter()
: filter_info_sub_(nullptr),
  mask_sub_(nullptr),
  state_event_pub_(nullptr),
  filter_mask_(nullptr),
  global_frame_("")
{
}

void ZoneParameterFilter::initializeFilter(
  const std::string & filter_info_topic)
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  global_frame_ = layered_costmap_->getGlobalFrameID();
  state_event_topic_ =
    node->declare_or_get_parameter<std::string>(
    name_ + "." + "state_event_topic", std::string("zone_filter_state"));

  // ⚑ How long a set_parameters round-trip may take before the target is
  // reported as not carrying the zone. <= 0 disables the deadline and restores
  // waiting forever, which is what this filter did before 2026-09-05 -- and
  // waiting forever reports success forever. Configurable because 5 s is a
  // guess about somebody else's robot; disabling it is a choice an integrator
  // should have to make on purpose.
  set_parameters_timeout_ =
    node->declare_or_get_parameter<double>(
    name_ + "." + "set_parameters_timeout", 5.0);
  if (set_parameters_timeout_ <= 0.0) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: set_parameters_timeout is %.3f (<= 0), so a target "
      "that never answers will never be detected and the zone will be reported "
      "as enforced on it indefinitely.",
      set_parameters_timeout_);
  }
  filter_info_topic_ = joinWithParentNamespace(filter_info_topic);
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Subscribing to \"%s\" topic for filter info...",
    filter_info_topic_.c_str());

  filter_info_sub_ = node->create_subscription<nav2_msgs::msg::CostmapFilterInfo>(
    filter_info_topic_,
    std::bind(&ZoneParameterFilter::filterInfoCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS());

  state_event_pub_ =
    node->create_publisher<std_msgs::msg::UInt8>(joinWithParentNamespace(state_event_topic_));
  state_event_pub_->on_activate();

  // ⚑ HUBOT — the human-decision surface, created beside the robot's one.
  decision_pub_ = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    joinWithParentNamespace(decision_topic_), rclcpp::QoS(10));
  decision_pub_->on_activate();

  loadStateConfig();
}

void ZoneParameterFilter::filterInfoCallback(
  const nav2_msgs::msg::CostmapFilterInfo::ConstSharedPtr & msg)
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!mask_sub_) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: Received filter info from %s topic.", filter_info_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: New costmap filter info arrived from %s topic. "
      "Updating old filter info.",
      filter_info_topic_.c_str());
    mask_sub_.reset();
  }

  if (msg->type != nav2_costmap_2d::ZONE_PARAMETER_FILTER) {
    RCLCPP_ERROR(
      logger_,
        "ZoneParameterFilter: CostmapFilterInfo type is %i, expected %i (ZONE_PARAMETER_FILTER)",
      msg->type, nav2_costmap_2d::ZONE_PARAMETER_FILTER);
    return;
  }

  if (msg->base != nav2_costmap_2d::BASE_DEFAULT || msg->multiplier != nav2_costmap_2d::MULTIPLIER_DEFAULT) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: base=%f and multiplier=%f are unused by this filter "
      "(state mapping is config-driven). Expected defaults (%f, %f).",
      msg->base, msg->multiplier, nav2_costmap_2d::BASE_DEFAULT, nav2_costmap_2d::MULTIPLIER_DEFAULT);
  }

  filter_info_received_ = true;
  mask_topic_ = joinWithParentNamespace(msg->filter_mask_topic);

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Subscribing to \"%s\" topic for filter mask...",
    mask_topic_.c_str());
  mask_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    mask_topic_,
    std::bind(&ZoneParameterFilter::maskCallback, this, std::placeholders::_1),
    nav2::qos::LatchedSubscriptionQoS(3));
}

void ZoneParameterFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::ConstSharedPtr & msg)
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());

  if (!filter_mask_) {
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: Received filter mask from %s topic.", mask_topic_.c_str());
  } else {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: New filter mask arrived from %s topic. Updating old filter mask.",
      mask_topic_.c_str());
    filter_mask_.reset();
  }

  filter_mask_ = msg;
}

void ZoneParameterFilter::loadStateConfig()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Obtain the node, parameter, and value for state entries
  auto read_entry =
    [&](const std::string & prefix) -> std::optional<StateParamEntry> {
      const std::string target_node =
        node->declare_or_get_parameter<std::string>(prefix + ".node", std::string(""));
      const std::string param_name =
        node->declare_or_get_parameter<std::string>(prefix + ".parameter", std::string(""));
      if (target_node.empty() || param_name.empty()) {
        RCLCPP_ERROR(
          logger_,
          "ZoneParameterFilter: '%s' must declare non-empty 'node' and 'parameter'.",
          prefix.c_str());
        return std::nullopt;
      }
      const std::string value_key = prefix + ".value";
      if (!node->has_parameter(value_key)) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.dynamic_typing = true;
        node->declare_parameter(value_key, rclcpp::ParameterValue{}, descriptor);
      }
      const rclcpp::Parameter value_param = node->get_parameter(value_key);
      if (value_param.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        RCLCPP_ERROR(logger_, "ZoneParameterFilter: '%s' is not set.", value_key.c_str());
        return std::nullopt;
      }
      return StateParamEntry{
      target_node, rclcpp::Parameter(param_name, value_param.get_parameter_value())};
    };

  const std::vector<std::string> state_names =
    node->declare_or_get_parameter<std::vector<std::string>>(
    name_ + ".states", std::vector<std::string>{});

  if (state_names.empty()) {
    RCLCPP_WARN(
      logger_,
      "ZoneParameterFilter: 'states' is empty; this filter will only handle "
      "state 0 (reset). Configure `states: [name_a, ...]` in YAML.");
  }

  for (const auto & state_name : state_names) {
    const std::string state_prefix = name_ + "." + state_name;

    const int64_t id_i64 =
      node->declare_or_get_parameter<int64_t>(state_prefix + ".id", 0);
    if (id_i64 <= 0 || id_i64 > 255) {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: state '%s' has id %ld outside the valid range "
        "[1, 255] (0 is reserved for reset); skipping.",
        state_name.c_str(), id_i64);
      continue;
    }
    const uint8_t state_id = static_cast<uint8_t>(id_i64);

    const std::vector<std::string> setpoint_names =
      node->declare_or_get_parameter<std::vector<std::string>>(
      state_prefix + ".setpoints", std::vector<std::string>{});

    std::vector<StateParamEntry> params_for_state;
    for (const auto & setpoint_name : setpoint_names) {
      if (auto entry = read_entry(state_prefix + "." + setpoint_name)) {
        params_for_state.push_back(std::move(*entry));
      }
    }

    if (params_for_state.empty()) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: state '%s' (id %u) declares no valid setpoints.",
        state_name.c_str(), state_id);
    }

    state_param_map_[state_id] = std::move(params_for_state);
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: state '%s' (id %u) -> %zu setpoint(s).",
      state_name.c_str(), state_id, state_param_map_[state_id].size());
  }

  // `nominal_defaults`: the baseline values restored on the state-0 reset.
  const std::vector<std::string> nominal_names =
    node->declare_or_get_parameter<std::vector<std::string>>(
    name_ + ".nominal_defaults", std::vector<std::string>{});
  for (const auto & nominal_name : nominal_names) {
    if (auto entry = read_entry(name_ + ".nominal_defaults." + nominal_name)) {
      nominal_defaults_[entry->target_node].push_back(entry->param);
    }
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu nominal default(s) loaded for state-0 reset.",
    nominal_names.size());

  // Warn on a state override with no matching nominal_default: the state-0
  // reset would not be able to restore that parameter.
  auto has_nominal = [this](const StateParamEntry & e) -> bool {
      const auto node_it = nominal_defaults_.find(e.target_node);
      if (node_it == nominal_defaults_.end()) {
        return false;
      }
      for (const auto & nominal : node_it->second) {
        if (nominal.get_name() == e.param.get_name()) {
          return true;
        }
      }
      return false;
    };
  for (const auto & [state_id, entries] : state_param_map_) {
    for (const auto & entry : entries) {
      if (!has_nominal(entry)) {
        RCLCPP_WARN(
          logger_,
          "ZoneParameterFilter: state id %u sets '%s' on '%s' but no matching "
          "nominal_defaults entry exists; state-0 reset will NOT restore it.",
          state_id, entry.param.get_name().c_str(), entry.target_node.c_str());
      }
    }
  }

  // Per-node client construction
  std::set<std::string> all_target_nodes;
  for (const auto & [_state_id, entries] : state_param_map_) {
    for (const auto & e : entries) {
      all_target_nodes.insert(e.target_node);
    }
  }
  for (const auto & [target_node, _params] : nominal_defaults_) {
    all_target_nodes.insert(target_node);
  }
  for (const auto & target_node : all_target_nodes) {
    param_clients_.emplace(
      target_node,
      std::make_shared<rclcpp::AsyncParametersClient>(
        node->get_node_base_interface(),
        node->get_node_topics_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        target_node));
  }
  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: %zu AsyncParametersClient(s) built at init.",
    param_clients_.size());

  // ⚑ A fault is a claim about a target we manage. If a reload stops naming a
  // target at all, we no longer have standing to assert anything about it, so
  // the claim is dropped -- and ONLY then. This is a deliberate operator act
  // (someone edited the configuration), not a recovery behaviour, which is the
  // distinction resetFilter() turns on.
  for (auto it = degraded_targets_.begin(); it != degraded_targets_.end(); ) {
    if (all_target_nodes.count(*it) == 0) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: dropping the degraded mark on '%s'; the reloaded "
        "configuration no longer names it as a target.",
        it->c_str());
      it = degraded_targets_.erase(it);
    } else {
      ++it;
    }
  }
  enforcement_degraded_ = !degraded_targets_.empty();
}

void ZoneParameterFilter::process(
  nav2_costmap_2d::Costmap2D & /*master_grid*/,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/,
  const geometry_msgs::msg::Pose & pose)
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());

  checkPendingParameterUpdates();

  if (!filter_mask_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask was not received");
    return;
  }

  geometry_msgs::msg::Pose mask_pose;
  if (!transformPose(global_frame_, pose, filter_mask_->header.frame_id, mask_pose)) {
    return;
  }

  unsigned int mask_robot_i, mask_robot_j;
  if (!nav2_util::worldToMap(
      filter_mask_, mask_pose.position.x, mask_pose.position.y,
      mask_robot_i, mask_robot_j))
  {
    if (state_initialized_ && current_state_ != 0) {
      RCLCPP_WARN(
        logger_,
        "ZoneParameterFilter: Robot outside filter mask; resetting to nominal defaults.");
      applyState(0);  // state 0 is always declared; it restores nominal_defaults.
      current_state_ = 0;
      mask_state_ = 0;
      mask_state_undeclared_ = false;
      // ⚑ CPP 2026-09-05: the latch is NOT cleared here.
      //
      // applyState(0) only ISSUES async set_parameters; the results arrive later
      // in checkPendingParameterUpdates(). Clearing the latch now published
      // `enforced: yes` on a restore nothing had confirmed — a success-shaped
      // value inside the feature built to abolish success-shaped values
      // (Sakichi Vision 14). It is the same defect this package names in
      // upstream's CostmapFilter::updateCosts(), which sets current_ = true
      // unconditionally after process() returns.
      //
      // The header's contract is the correct one and now the code matches it:
      // the flag latches, and only a reload clears it, because a reload is the
      // one event that makes the configuration it referred to meaningless.
      publishDecision("left the filter mask; restoring nominal defaults");
    }
    return;
  }

  const int8_t mask_data = getMaskData(filter_mask_, mask_robot_i, mask_robot_j);
  if (mask_data < 0) {
    // mask_data < 0 is OCC_GRID_UNKNOWN; don't change state on an unknown cell.
    RCLCPP_WARN_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: Filter mask cell [%u, %u] is unknown; not changing state.",
      mask_robot_i, mask_robot_j);
    return;
  }

  const uint8_t new_state = static_cast<uint8_t>(mask_data);

  if (state_initialized_ && new_state == current_state_) {
    return;  // No change.
  }

  // ⚑ CPP 2026-09-05 — P-1. applyState() USED TO RETURN void AND THROW HERE.
  // The throw is gone; returning false without this guard would have been only
  // half the fix, and the worse half. Control still reached the assignment
  // below, so the filter recorded a state it had NOT applied: the previous
  // zone's overrides stayed on the targets, `current_state_` named a state that
  // is not in `state_param_map_`, and the NEXT transition's N-only reset block
  // (applyState, below) then missed its lookup and reset NOTHING -- carrying the
  // old zone's limits into a zone that never declared them, until some later
  // exit-to-nominal happened to restore everything.
  //
  // So the state is committed only when it was actually applied. When it was
  // not, `current_state_` keeps naming the state whose values are genuinely in
  // force, which is the truth, and `mask_state_` carries where the mask says we
  // are. Those two disagreeing IS the fault, and both are published.
  if (!applyState(new_state)) {
    const bool newly_undeclared = !mask_state_undeclared_ || mask_state_ != new_state;
    mask_state_ = new_state;
    mask_state_undeclared_ = true;
    if (newly_undeclared) {
      // Only on entry to the condition: process() runs at the costmap rate and
      // the robot can sit on this cell indefinitely.
      publishDecision(
        "mask state " + std::to_string(static_cast<int>(new_state)) +
        " is not declared; no zone limits were applied for it");
    }
    return;
  }
  mask_state_ = new_state;
  mask_state_undeclared_ = false;

  // ⚑ FSE 2026-09-05 — THE ASSIGNMENT MOVED ABOVE THE PUBLISHES, AND THAT IS
  // THE WHOLE FIX.
  //
  // publishDecision() renders `current_state_`. It used to run BEFORE this
  // assignment, so on entering zone 3 the human-decision surface published
  // `zone_state: 0` and "Outside any zone; nominal defaults are in force." —
  // the OPPOSITE of the truth, at the exact moment the comment below says a
  // human decision is due. The first transition also published
  // `configured: not yet`, because state_initialized_ was still false.
  //
  // Order is safe: applyState() is the only reader of the previous
  // current_state_/state_initialized_ (it uses them to compute the N-only
  // resets) and has already returned.
  current_state_ = new_state;
  state_initialized_ = true;

  if (state_event_pub_) {
    auto event_msg = std::make_unique<std_msgs::msg::UInt8>();
    event_msg->data = new_state;
    state_event_pub_->publish(std::move(event_msg));
  }
  // ⚑ HUBOT: the robot gets the byte; the person gets the basis, in the
  // same breath. A transition is exactly when a human decision is due.
  publishDecision("zone transition");
}

bool ZoneParameterFilter::applyState(uint8_t new_state)
{
  if (new_state == 0) {
    resetToNominal();
    RCLCPP_INFO(logger_, "ZoneParameterFilter: Entered state 0 (reset to nominal).");
    return true;
  }

  auto it = state_param_map_.find(new_state);
  if (it == state_param_map_.end()) {
    // ⚑ CPP 2026-09-05 — THE SECOND ABORT PATH, closed by the same rule as the
    // first. This THREW; process() calls applyState() bare and
    // CostmapFilter::updateCosts() is bare (costmap_filter.cpp:124-134), so a
    // mask cell carrying an id no state declares killed the navigation node --
    // the same chain checkPendingParameterUpdates() was fixed to stop. Reached
    // by MASK DATA, not by configuration: a single mis-painted pixel is enough.
    // test/zpf_survival_probe.cpp mode `unknown-state` observed exactly that,
    // out-of-process, before this change: "DIED by signal 6 (Aborted)".
    //
    // ⚑ THE CONVENTION CLAIM, CORRECTED. An earlier draft of this comment said
    // the three sibling filters "not one of them throws". That is false and a
    // maintainer refutes it with one grep: speed_filter.cpp throws at :66 and
    // :168, keepout_filter.cpp at :66, :101 and :146, binary_filter.cpp at :66
    // and :109 -- seven sites. The TRUE claim is stronger and is the one that
    // bears on this line: EVERY ONE of those seven is the same node-lock guard
    // (`throw std::runtime_error{"Failed to lock node"}`) in initializeFilter()
    // or a subscription callback. NOT ONE of them throws from process(), and
    // NOT ONE throws on a data-dependent condition. Three independent sites,
    // one pattern: in this filter family, the data path does not throw.
    RCLCPP_ERROR_THROTTLE(
      logger_, *(clock_), 2000,
      "ZoneParameterFilter: mask state %u is not declared under the filter's "
      "`states` list; NO zone limits were applied for it and the parameters in "
      "force are still state %u's. Declare a state with id %u, or correct the "
      "mask.",
      new_state, current_state_, new_state);
    return false;
  }

  std::set<std::pair<std::string, std::string>> m_keys;
  for (const auto & entry : it->second) {
    m_keys.emplace(entry.target_node, entry.param.get_name());
  }

  // Reset params touched by the previous state N but not the destination M
  // back to nominal_defaults before applying M's overrides. This preserves
  // the invariant that all params equal state-0 defaults except those
  // specifically set in the active state.
  // Reset params touched by the previous state N but not the destination M
  // back to nominal_defaults before applying M's overrides.
  std::map<std::string, std::vector<rclcpp::Parameter>> reset_per_node;
  if (state_initialized_ && current_state_ != 0) {
    auto prev_it = state_param_map_.find(current_state_);
    if (prev_it != state_param_map_.end()) {
      for (const auto & entry : prev_it->second) {
        const auto key = std::make_pair(entry.target_node, entry.param.get_name());
        if (m_keys.count(key) > 0) {
          continue;  // M will set this param; reset is wasted work.
        }
        const auto node_it = nominal_defaults_.find(entry.target_node);
        if (node_it == nominal_defaults_.end()) {
          // No nominal_defaults were declared for this node, so this param has
          // nothing to reset to; it keeps state N's value. Warned at config-load.
          continue;
        }
        for (const auto & nominal : node_it->second) {
          if (nominal.get_name() == entry.param.get_name()) {
            reset_per_node[entry.target_node].push_back(nominal);
            break;
          }
        }
      }
    } else {
      RCLCPP_ERROR(
        logger_,
        "ZoneParameterFilter: current state %u is not in state_param_map_ "
        "(should have been recorded when the state was applied).",
        current_state_);
    }
  }

  // Batch per target node (one set_parameters call per node).
  std::map<std::string, std::vector<rclcpp::Parameter>> per_node_params;
  for (const auto & entry : it->second) {
    per_node_params[entry.target_node].push_back(entry.param);
  }

  // Submit the N-only resets before M's overrides.
  size_t reset_count = 0;
  for (const auto & [target_node, params] : reset_per_node) {
    issueAsyncSetParameters(target_node, params);
    reset_count += params.size();
  }

  for (const auto & [target_node, params] : per_node_params) {
    issueAsyncSetParameters(target_node, params);
  }

  RCLCPP_INFO(
    logger_,
    "ZoneParameterFilter: Entered state %u (reset %zu N-only parameter(s); "
    "applied %zu parameter(s) across %zu node(s)).",
    new_state, reset_count, it->second.size(), per_node_params.size());
  return true;
}

void ZoneParameterFilter::resetToNominal()
{
  for (const auto & [target_node, params] : nominal_defaults_) {
    issueAsyncSetParameters(target_node, params);
  }
}

void ZoneParameterFilter::markTargetDegraded(
  const std::string & target_node, const std::string & why)
{
  const bool first = degraded_targets_.insert(target_node).second;
  unconfirmed_targets_.erase(target_node);
  enforcement_degraded_ = !degraded_targets_.empty();
  if (first) {
    RCLCPP_ERROR(
      logger_,
      "ZoneParameterFilter: target '%s' is NOT carrying zone %u's values: %s",
      target_node.c_str(), current_state_, why.c_str());
    publishDecision("target '" + target_node + "' not enforcing: " + why);
  }
}

void ZoneParameterFilter::markTargetHealthy(const std::string & target_node)
{
  unconfirmed_targets_.erase(target_node);
  // ⚑ The ONLY retraction path. A fault against a target is a claim about that
  // target's live parameter value, and the only thing that refutes it is that
  // target answering successfully. Not a costmap clear, not a reload, not time.
  if (degraded_targets_.erase(target_node) > 0) {
    enforcement_degraded_ = !degraded_targets_.empty();
    RCLCPP_INFO(
      logger_,
      "ZoneParameterFilter: target '%s' accepted a set and is no longer "
      "reported degraded.%s",
      target_node.c_str(),
      enforcement_degraded_ ? " Other targets are still degraded." : "");
    publishDecision("target '" + target_node + "' accepted a set");
  }
}

void ZoneParameterFilter::issueAsyncSetParameters(
  const std::string & target_node,
  const std::vector<rclcpp::Parameter> & params)
{
  auto client_it = param_clients_.find(target_node);
  if (client_it == param_clients_.end()) {
    // ⚑ This used to log and return, latching nothing -- so a target with no
    // client produced a completely silent unenforcement while publishDecision()
    // went on reporting `enforced: yes`. Same success-shaped value, different
    // door.
    markTargetDegraded(
      target_node, "no parameter client exists for it (config-load did not build one)");
    return;
  }

  // ⚑ kMaxPendingSets, referenced ZERO times before 2026-09-05 while its
  // comment claimed it bounded a never-answering target. It cannot do that --
  // one silent set sits at a count of one forever. The deadline in
  // checkPendingParameterUpdates() does that job; this bounds GROWTH against a
  // target answering slower than the costmap rate, and the oldest victim is
  // reported rather than dropped quietly.
  while (pending_sets_.size() >= kMaxPendingSets) {
    const std::string dropped = pending_sets_.front().target_node;
    pending_sets_.erase(pending_sets_.begin());
    markTargetDegraded(
      dropped,
      "more than " + std::to_string(kMaxPendingSets) +
      " sets are in flight; the oldest was discarded unanswered");
  }

  unconfirmed_targets_.insert(target_node);
  pending_sets_.push_back(
    PendingSet{
      target_node,
      client_it->second,
      client_it->second->set_parameters(params),
      clock_->now()});
}

void ZoneParameterFilter::checkPendingParameterUpdates()
{
  // A silently-swallowed set failure would leave the robot on the value the
  // safety zone tried to change, so every failure is surfaced. It is surfaced
  // by marking the TARGET degraded and continuing, never by throwing: this
  // function is called from the top of process(), process() is reached from
  // CostmapFilter::updateCosts(), which is bare, from LayeredCostmap, which has
  // no try/catch anywhere -- so a throw here is std::terminate and the
  // navigation node dies. Sakichi Vision 20: the halt must cost less than the
  // defect.
  // wait_for(0s) polls without blocking the costmap update loop.
  const rclcpp::Time now = clock_->now();
  auto it = pending_sets_.begin();
  while (it != pending_sets_.end()) {
    if (it->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      // ⚑ F-2, THE FAILURE THAT LOOKS EXACTLY LIKE SUCCESS. Before 2026-09-05
      // this branch was a bare `++it`: a future that never becomes ready was
      // never examined, so there was no failure, no latch, and the filter went
      // on publishing `enforced: yes` and "Zone N is in force" forever while
      // the zone's limits were on nothing. The target node crashed, was never
      // brought up, is still configuring, or the namespace in the YAML has a
      // typo -- every one of those produces silence, and silence was reported
      // as enforcement. A rejection is the EASY case; somebody answered.
      if (set_parameters_timeout_ > 0.0 &&
        (now - it->issued_at).seconds() > set_parameters_timeout_)
      {
        const std::string target = it->target_node;
        it = pending_sets_.erase(it);
        markTargetDegraded(
          target,
          "no answer within " + std::to_string(set_parameters_timeout_) +
          "s of the set being issued; an unanswered request is not a "
          "successful one");
        continue;
      }
      ++it;
      continue;
    }

    // Copy the shared state out before erasing: the copy keeps the value that
    // get() returns a reference to alive for this iteration, and erasing first
    // means a service-side exception rethrown by get() surfaces exactly once.
    const auto ready_future = it->future;
    const std::string target = it->target_node;
    it = pending_sets_.erase(it);

    // ⚑ HUBOT CHANGE — the whole reason this package exists.
    //
    // Upstream this block THREW. checkPendingParameterUpdates() is called at the
    // top of process(); process() is reached from CostmapFilter::updateCosts(),
    // which is bare, from LayeredCostmap, which has no try/catch anywhere. So a
    // failed parameter set left the costmap update thread with no handler:
    // std::terminate, and the navigation node dies.
    //
    // The upstream INTENT is right and is kept verbatim in the comment above: a
    // silently-swallowed failure would leave the robot on the value the safety
    // zone tried to change. But a robot whose nav stack aborts is not safer than
    // one that degrades loudly and keeps navigating. Sakichi Vision 20 — the halt
    // must cost less than the defect.
    //
    // So: never silent, never fatal. Every failure is logged at ERROR with its
    // reason, the filter latches a degraded flag, and it keeps running so the
    // rest of the stack can decide.
    try {
      const auto & results = ready_future.get();
      bool all_ok = true;
      for (const auto & r : results) {
        if (!r.successful) {
          all_ok = false;
          // ⚑ The message names the target now. It used to say "on that
          // target" and never say which -- an integrator reading the log of a
          // multi-target zone could not tell what to go and look at.
          markTargetDegraded(target, "set_parameters rejected it: " + r.reason);
        }
      }
      if (all_ok) {
        markTargetHealthy(target);
      }
    } catch (const std::exception & ex) {
      // A service-side exception rethrown by get(). Same rule.
      markTargetDegraded(target, std::string("set_parameters threw: ") + ex.what());
    }
  }

  // ⚑ The good news needs a publish too. Without this the surface says
  // `pending` once and falls silent for a filter that is working perfectly.
  publishDecisionOnStatusChange();
}

// ⚑ HUBOT — THE HUMAN-DECISION SURFACE.
//
// Komada-voice 2026-09-05: "zone parameter is for robot. not for human. but we
// need it for human decision. build it."
//
// Upstream publishes a bare state byte. A byte is a robot's input: it selects a
// parameter set and nothing about it is a reason. A person deciding whether to
// rely on the zone needs the BASIS — which zone, what it changed, on which
// targets, and whether it actually took effect. The last of those is the one a
// number can never carry, and it is the one that matters: upstream expressed
// "I could not enforce this zone" by killing the process.
//
// diagnostic_msgs/DiagnosticArray is chosen because it is the ROS-native way to
// say something to a person, it needs no new interface package, and every
// operator tool already renders it. The robot's UInt8 topic is untouched: this
// ADDS a surface, it does not replace one.
std::string ZoneParameterFilter::enforcementToken() const
{
  if (enforcement_degraded_ || mask_state_undeclared_) {
    return "NO";
  }
  return unconfirmed_targets_.empty() ? "yes" : "pending";
}

void ZoneParameterFilter::publishDecisionOnStatusChange()
{
  const std::string token = enforcementToken();
  if (token == last_published_token_) {
    return;
  }
  // ⚑ Caught by CPP_N3 on its second run, and it is the SAME defect a third
  // time. Before a state has ever been applied there is nothing outstanding
  // and nothing degraded, so the token computes to "yes" -- and the filter
  // announced `enforced: yes` at startup, having set nothing, confirmed
  // nothing, and not yet read the mask. A reassuring word about work not done
  // is the exact shape N-3 is about. The cache is updated silently so the
  // first REAL transition still registers as a change.
  if (!state_initialized_) {
    last_published_token_ = token;
    return;
  }
  publishDecision("enforcement is now: " + token);
}

void ZoneParameterFilter::publishDecision(const std::string & detail)
{
  if (!decision_pub_) {
    return;
  }
  last_published_token_ = enforcementToken();
  diagnostic_msgs::msg::DiagnosticStatus st;
  st.name = "zone_parameter_filter";
  st.hardware_id = global_frame_;

  // ⚑ CPP 2026-09-05 — N-3. `enforced` IS THREE-VALUED NOW, and the middle
  // value is the finding. This block used to publish `enforced: yes` and
  // "Zone N is in force." on the SAME CYCLE the async sets were issued, with
  // zero confirmations in hand -- while the header, in this same package,
  // states that reporting `enforced: yes` on an unconfirmed restore is exactly
  // the success-shaped value this feature exists to abolish. The code applied
  // its own principle on the leave path and violated it on the enter path.
  // `pending` is not a weaker `yes`; an operator must read it as `NO` until it
  // resolves one way or the other.
  const bool unconfirmed = !unconfirmed_targets_.empty();
  if (enforcement_degraded_) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message =
      "Zone " + std::to_string(static_cast<int>(current_state_)) +
      " is NOT being enforced on at least one target. Decide as if the zone's "
      "limits are not applied.";
  } else if (mask_state_undeclared_) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    st.message =
      "The mask reports zone " + std::to_string(static_cast<int>(mask_state_)) +
      " at the robot's pose and no state with that id is configured. No limits "
      "were applied for it; what is in force is still zone " +
      std::to_string(static_cast<int>(current_state_)) +
      "'s. Decide as if this zone has no limits.";
  } else if (unconfirmed) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    st.message =
      "Zone " + std::to_string(static_cast<int>(current_state_)) +
      " has been REQUESTED on " + std::to_string(unconfirmed_targets_.size()) +
      " target(s) and none of them has confirmed yet. Do not rely on its "
      "limits until this reads yes.";
  } else if (current_state_ == 0) {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message = "Outside any zone; nominal defaults are in force.";
  } else {
    st.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message =
      "Zone " + std::to_string(static_cast<int>(current_state_)) +
      " is in force.";
  }

  const auto add = [&st](const std::string & k, const std::string & v) {
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = k;
      kv.value = v;
      st.values.push_back(kv);
    };
  add("zone_state", std::to_string(static_cast<int>(current_state_)));
  // ⚑ The state whose values are in force and the state the mask reports are
  // different facts, and they diverge exactly when something is wrong. One
  // number could never carry that, which is why both are published.
  add("mask_state", std::to_string(static_cast<int>(mask_state_)));
  add("enforced", last_published_token_);
  add("configured", state_initialized_ ? "yes" : "not yet");
  add("unconfirmed_targets", std::to_string(unconfirmed_targets_.size()));
  add("degraded_targets", std::to_string(degraded_targets_.size()));
  add("pending_parameter_sets", std::to_string(pending_sets_.size()));
  add("targets", std::to_string(param_clients_.size()));
  if (!detail.empty()) {
    add("event", detail);
  }

  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = clock_->now();
  arr.status.push_back(st);
  decision_pub_->publish(arr);
}

void ZoneParameterFilter::resetFilter()
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());

  filter_info_sub_.reset();
  mask_sub_.reset();
  if (state_event_pub_) {
    state_event_pub_->on_deactivate();
    state_event_pub_.reset();
  }
  if (decision_pub_) {
    decision_pub_->on_deactivate();
    decision_pub_.reset();
  }

  filter_mask_.reset();
  filter_info_received_ = false;
  state_initialized_ = false;
  current_state_ = 0;
  mask_state_ = 0;
  mask_state_undeclared_ = false;

  // ⚑ CPP 2026-09-05 — N-1, AND THE FIX IS THE OPPOSITE OF THE ONE THAT WAS
  // HERE. This line used to read `enforcement_degraded_ = false;`, justified in
  // the header by "a reload clears it because the configuration it referred to
  // is gone". Both halves of that were false.
  //
  // (1) THE CONFIGURATION WAS NOT GOING ANYWHERE. `state_param_map_`,
  //     `nominal_defaults_` and `param_clients_` had no `.clear()` anywhere in
  //     this translation unit -- zero occurrences each. And
  //     CostmapFilter::reset() (costmap_filter.cpp:103) is
  //     `resetFilter(); initializeFilter(...); setCurrent(false);` -- so
  //     loadStateConfig() re-ran over those same containers, and
  //     `nominal_defaults_[node].push_back(...)` APPENDS. Every reload
  //     duplicated every nominal default, without bound, and kept clients for
  //     targets the new configuration had dropped. The clears below are what
  //     the header always said was happening.
  //
  // (2) THE FLAG IS NOT A CLAIM ABOUT THE CONFIGURATION. It is a claim about a
  //     TARGET NODE'S LIVE PARAMETER VALUE, which no reload of a costmap filter
  //     reaches. And the path is routine, not exceptional: ClearEntireCostmap
  //     appears in SEVEN of nav2's default behaviour trees and arrives here via
  //     Costmap2DROS::resetLayers() (costmap_2d_ros.cpp:719). An ordinary
  //     recovery erased the fault flag while the fault stood -- and with it,
  //     the one signal an integrator had.
  //
  // So `degraded_targets_` and `enforcement_degraded_` SURVIVE. They are
  // retracted by markTargetHealthy(), on a set that actually comes back
  // successful, and by nothing else. Pending sets DO go: they belong to the
  // discarded configuration, and leaving them was its own defect -- their
  // results would have re-latched against a configuration that no longer
  // existed.
  pending_sets_.clear();
  unconfirmed_targets_.clear();
  last_published_token_.clear();  // a cache; a stale one would suppress a publish
  state_param_map_.clear();
  nominal_defaults_.clear();
  param_clients_.clear();
}

bool ZoneParameterFilter::isActive()
{
  std::lock_guard<nav2_costmap_2d::CostmapFilter::mutex_t> guard(*getMutex());
  return filter_mask_ != nullptr;
}

}  // namespace hubot

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(hubot::ZoneParameterFilter, nav2_costmap_2d::Layer)
