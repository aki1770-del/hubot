# Changelog

All notable changes to `hubot`. Format follows Keep a Changelog; versions follow SemVer.

## [Unreleased]

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
See **Read this before you deploy it** in `README.md`. In particular, this filter
cannot signal to the navigation stack that its output is untrustworthy, and it has
not been run on hardware.
