# Changelog

All notable changes to `hubot`. Format follows Keep a Changelog; versions follow SemVer.

## [Unreleased]

### Added
- **A positive liveness signal, so that silence on `zone_decision` reads as "I am not
  watching" rather than "you are safe."** A wall timer on the node — deliberately not on
  the costmap update loop — publishes every `liveness_period` whether or not anything
  changed, and declares the costmap stopped after `costmap_silence_timeout` without a
  `process()` call. New `values`: `watching`, `costmap_age_s`, `report_seq`,
  `report_period_s`, `valid_for_s`.
- The timer also runs the pending-parameter-set check, so **the `set_parameters` deadline
  fires and `pending` resolves while the costmap is stopped.** Previously neither could.
- New parameters `liveness_period` (default `1.0` s) and `costmap_silence_timeout`
  (default `2.0` s). Either `<= 0` disables its part of the mechanism, with a startup
  warning naming what is given up — the same convention `set_parameters_timeout` uses.
- `DiagnosticStatus::STALE` is now emitted, for the not-watching condition only.
- `test/costmap_silence_liveness_test.cpp` — six cases whose oracle **distinguishes a
  stopped filter from a running-and-quiet one**, driving `updateCosts()` from one thread
  while the executor spins on another. Two of the six are **negative controls that require
  the original defect to reappear** when the mechanism is switched off; both were verified
  passing against the unmodified filter before the fix existed, and one case exercises
  `resetFilter()` concurrently with a live timer.

### Changed
- ⚑ **BREAKING (vocabulary): `enforced` has a fourth value, `unknown`**, meaning the
  costmap has stopped calling the filter — nothing failed, and nothing is being checked.
  Made now because this package has no remotes and no consumer can hold the old set.
- ⚑ **BREAKING (contract): the documented consumer rule is a whitelist.** It was *"refuse
  to drive on `enforced: NO` or `pending`"*; it is now **"proceed only on `enforced:
  yes`"**. The blacklist form fails open — a consumer coded literally against it would
  have read `unknown` as permission.
- A **disabled** filter now reports `watching: NO`. `CostmapFilter::updateCosts()` returns
  early when `enabled_` is false (`costmap_filter.cpp:128-130`), and `enabled_` is flipped
  by the base class's `<name>/toggle_filter` service (`:82-86`), so a disabled filter
  previously looked identical to an enforced zone. This door was not named in any prior
  bound.

### Fixed
- **AoU-5 and the matching README bound were false and are replaced in place, with the old
  text kept.** They said the result check runs from `process()` "and nowhere else, because
  a costmap filter owns no timer", and offered "check it" as the countermeasure —
  detection by the reader. `Layer::node_` is a live protected handle (`layer.hpp:186`) and
  `Costmap2DROS` always runs `map_update_thread_` separately from the thread its node is
  spun on (`costmap_2d_ros.cpp:314`; all three non-test construction sites in nav2:
  `controller_server.cpp:72`, `planner_server.cpp:78`, `costmap_2d_node.cpp:47`).

### Known limitation — stated, not closed
- **If the node itself dies, the timer dies with it and the last message stands.** Nothing
  running inside a process can announce that process's own death. Every message now carries
  a declared expiry (`report_period_s`, `valid_for_s`) and a monotonic `report_seq` a frozen
  clock cannot fake, which is strictly better than a bare header stamp — **and is still a
  check the reader must perform.** The complete answer is a `DEADLINE` QoS or a
  `diagnostic_aggregator` staleness rule **in the consumer's process. NOT IMPLEMENTED HERE
  AND NOT MEASURED.**

## [0.1.0]

### Added
- `hubot::ZoneParameterFilter` — a `nav2_costmap_2d` costmap filter that applies
  configured ROS parameter overrides to target nodes based on the mask value at the
  robot's pose.
- `zone_decision` (`diagnostic_msgs/DiagnosticArray`) — published on every zone
  transition and every enforcement failure, carrying `level`, an operator-readable
  `message`, and the `zone_state` / `mask_state` / `enforced` / `configured` /
  `unconfirmed_targets` / `degraded_targets` / `pending_parameter_sets` / `targets` /
  `event` values.
- Three-valued `enforced` (`yes` / `pending` / `NO`), so an unanswered parameter set
  is distinguishable from a confirmed one.
- A deadline on outstanding parameter sets, so a target that never answers resolves
  to `NO` rather than remaining outstanding indefinitely.

### Changed
- A rejected, failed, or unanswered parameter set is logged at `ERROR`, latches
  `enforcementDegraded()`, and leaves the costmap running, instead of propagating an
  exception out of `process()`.
- A mask value naming a state with no configuration is reported through
  `zone_decision` as `enforced: NO` rather than raising.

### Fixed
- A mask value naming an undeclared state was recorded as the current state, so the
  next transition's reset did not fire and the previous zone's limits persisted into
  a zone that had not requested them.
- `resetFilter()` cleared the degraded flag while the configuration it referred to
  remained loaded; `CostmapFilter::reset()` then re-ran the configuration load over
  it, and `nominal_defaults_` accumulated duplicate entries on each reset.
- `enforced: yes` was published on the same cycle the parameter sets were issued,
  before any confirmation had been received.
- `kMaxPendingSets` was declared but never referenced, and could not have bounded a
  single unanswered set in any case; a deadline replaces it.

### Known limitations
See **Read this before you deploy it** in `README.md`. In particular:

- ⚑ **This does not build against a released nav2.** `src/zone_parameter_filter.cpp:119`
  needs `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, which is absent from tags `1.5.0` and
  `1.5.1` and present only on branches `main` and `lyrical`. Measured 2026-09-06 against
  tag `1.5.1` (`a6354f3f`) from a clean workspace: compile error. Verified to build and
  test green against upstream `lyrical` HEAD `6f23b11c` only.
- This filter cannot signal to the navigation stack that its output is untrustworthy.
- It has not been run on hardware.
