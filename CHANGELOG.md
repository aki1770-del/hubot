# Changelog

All notable changes to `hubot`. Format follows Keep a Changelog; versions follow SemVer.

## [Unreleased]

### Added
- **`test/maintainer_findings_test.cpp` — a harness built to the specification of a review we
  already received.** nav2's maintainer found something on all three rounds of review of the
  upstream sibling of this filter, and asked, plainly, *"which maybe you can catch yourself? Not
  sure why I'm catching them but you're not."* His findings are the specification here.
  Six cases. **Four failed on the parent commit before any fix existed** and are recorded with
  that output; one is a negative control that must stay green; one is coverage for a breaking
  change that had none.
  - ⚑ **Why 486 passing tests caught none of them: every one of his findings is about a
    SEQUENCE — a lifecycle transition or a message arrival order — and not one is "this function
    computes the wrong value."** A value oracle cannot fail on a sequence defect.
  - ⚑ **And why our own suite could not have caught them either: the setup helper waited the
    defect out.** It called `resetFilter()` and then spun until `isActive()`, which is
    `filter_mask_ != nullptr` — so it blocked until the latched mask had been redelivered, which
    is precisely the window his blocking findings live in. **The new fixture has no
    wait-for-active after a reset**; it calls `reset()` (the production entry point) and asserts
    immediately, inside the window.
- `doc/SPEC_COVERAGE.md` §4 now carries **a per-finding verdict with its evidence** — occurs
  here / cannot occur here with a named reason / unverified — because *"does not apply here"* is
  the sentence under which a defect survives a sweep.

### Fixed
- ⚑ **The package invited an install it cannot deliver, and the disqualifying fact was 220 lines
  below the invitation.** The README's `find_package(hubot REQUIRED)` block sat near the top with
  nothing between it and a reader; the release gap was recorded honestly, but in the bounds
  section near the end. **An accurate fact a reader reaches after acting is not a disclosure.**
  The gap now precedes the instructions, says plainly that the package cannot be installed today,
  and hands the reader a one-line check to run against their own `nav2_costmap_2d` rather than a
  claim to trust.
- ⚑ **`doc/SPEC_COVERAGE.md` marked *"installable without an upstream merge"* as **FIT**.** It is
  a **GAP**, and the evidence column proved the wrong thing: presence of a `pluginlib` export says
  nothing about whether the source compiles. The row asked whether an integrator can install this
  and was answered with whether we had packaged it.
- ⚑ **`enforced: yes` and `watching: yes` from a filter that had never been driven once.**
  `initializeFilter()` runs at configure and starts the heartbeat; the costmap's update thread is
  not created until activate. In that gap — **every ordinary bringup, and re-opened by every
  routine `ClearEntireCostmap`** — the filter published level `OK`, `enforced: yes`,
  `watching: yes`, having read no mask, applied no state and confirmed nothing. A consumer could
  not distinguish it from a zone confirmed on every target.
  - ⚑ **This is the fourth generation of one defect family, and the first three were all fixed
    at a CALLER:** yes-on-issue, then yes-at-startup, then yes-during-silence. Each fix went into
    a call site, and each new call site then arrived without it — `livenessTick()` was the new
    caller this time. **This one is fixed in the VALUE.** `notWatching()` is a single predicate
    that both `enforcementToken()` and the `watching` field derive from, so a future caller
    cannot compute around it.
  - The discriminator already existed: `ever_processed_` was written in three places and read in
    exactly one — inside a message string, never in the token or the level. It is consulted by
    the value now.
  - **Not-yet-driven and stopped deliberately report identically**, because to a consumer they
    are the same fact: nobody is watching.
- ⚑ **A routine clear was silent on the surface that exists to report it.** `resetFilter()`
  destroyed both publishers at the top and mutated every piece of state a reader depends on
  below — mask, configuration, current state, outstanding sets — so nothing announced the
  transition. **This is the ordering defect nav2's maintainer named on the upstream filter,
  present here independently.** `resetFilter()` now clears state, publishes one farewell
  describing the cleared state, and destroys the publishers last.
- ⚑ **`pending` resolved to `yes` without any confirmation ever arriving.** `resetFilter()`
  discards `pending_sets_` and `unconfirmed_targets_` while those requests are still on the wire,
  so the filter forgot sets it had issued and could never learn their outcome — and the token it
  published moved to the reassuring value by discarding the evidence that refuted it. Now covered
  by the not-watching gate, so a clear reports `unknown` rather than `yes`.
- The not-watching message no longer says *"Zone N's limits were last requested"* when nothing
  was ever requested; the never-driven case gets its own sentence, true in its own branch.
- ⚑ **`test/safety_invariants_static.sh` was wrong half the time, and nothing ran it.** It had
  been committed since 2026-09-05 and was referenced by no CMake target, no hook and no cron —
  so it had never fired. Run by hand on unchanged source that satisfies every invariant, it
  reported **FAIL on 6 of 12 consecutive runs.** Cause, probed directly: two checks ended in
  `| grep -q` under `set -o pipefail`; `grep -q` exits on its first match, SIGPIPEs the upstream
  `grep`, and `pipefail` promotes the resulting 141 to the pipeline status (`rc=141` on 7 of 12
  probe runs). ⚑ **The failure mode was inverted** — the more obviously the invariant held, the
  likelier it was reported violated. Both sites now count with `grep -c`, which consumes its
  input. **20/20 green on the true source; 5/5 red against each of two mutants** (one removing
  the only code reference to `kMaxPendingSets`, one removing `pending_sets_.clear()`). It is now
  registered with CTest, so it runs in `colcon test`.

### Changed
- `valid_for_s`'s `2.5` factor is now a named constant with its justification, and the README
  **defines the field for a reader** rather than only listing it.
- The README calls `valid_for_s` **"a declared expiry"** rather than *"a promise with a number in
  it."* An advisory component may not make a promise about timing; the information is the same.
- ⚑ **Corrected on this package's own face, rather than quietly:** `doc/SPEC_COVERAGE.md` said
  *"none of the six is present in Hubot."* **Two of them are, one character-for-character**, and
  that sentence is exactly the shape that lets a defect survive a sweep. `GAP-0` — *"THE PACKAGE
  AS COMMITTED WILL NOT BUILD"* — is also superseded: every row of it is now false, re-measured
  green. Both are corrected in place with the old text kept, because the old text is the reason
  the new text exists.

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
