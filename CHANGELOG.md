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
- ⚑ **We namespace-joined our own four topic names and none of the integrator's.** `Layer::joinWithParentNamespace()` was applied at `:130`, `:142`, `:147`, `:215` — every name we own — and to **zero** of the `node:` fields an integrator writes. Under a namespaced launch a relative `node: controller_server` therefore resolved against the root while our topics resolved against the parent; an `AsyncParametersClient` was built for a node that does not exist; and **the missing-client guard at `issueAsyncSetParameters()` could not fire, because the key it looks up is the same key the phantom was built under.** The set simply never landed. Their names now get the same loom as ours.
  - **Absolute names are unaffected** — nav2's `Layer::joinWithParentNamespace()` returns any name beginning with `/` unchanged (`layer.cpp:90-96`, read at release tag `1.5.1`), which is why every documented example kept working and the fault stayed invisible to us.
  - The join is deliberately placed **after** the empty-check: it maps `""` to `"<parent>/"`, which is non-empty, and would otherwise have silently defeated the guard that rejects a missing `node:`.
  - ⚑ The requirement appeared **zero times** in the README. That was part of the same defect and is fixed with it.
- ⚑ **The reason for a failure was published on exactly one message and then taken away.** `event` was gated on a non-empty `detail`, and the 1 Hz heartbeat passes `std::string()` on every tick where nothing flipped — so `target 'X' not enforcing: <why>` survived a single message over volatile transport while the condition it explained persisted, leaving `enforced: NO` and a non-zero `degraded_targets` with nothing saying why. The last non-empty reason is now carried forward. **No new field**: it is self-correcting, because every resolution path publishes its own non-empty detail which replaces it.
- ⚑ **One documented line of YAML reinstated the reassuring value.** `costmap_silence_timeout: 0` made `costmap_silent_` unreachable, so a filter whose costmap died an hour ago published `watching: yes`, `enforced: yes`, level `OK` forever. A fallback budget of `liveness_period x kValidForPeriods` now applies when the explicit detector is switched off. The explicit path is byte-for-byte unchanged for any positive timeout.
  - ⚑ **THE PROPOSED FIX WAS TRIED FIRST AND IS REFUTED BY MEASUREMENT.** `sotif_gate_inertness_test.cpp` SC-1 proposes, in its own comment, `notWatching() { return !ever_processed_ || costmap_silent_ || costmap_silence_timeout_ <= 0.0; }`. Implemented exactly as written it turns **SC-1 green and SC-2 red** — and SC-2 is that same file's non-optional negative control, which states *"if SC-2 ever goes red, SC-1 passing means nothing."* A constant disjunct cannot distinguish a driven costmap from a stopped one, so it buys SC-1 by making the filter never assert `yes` at all: the degenerate pass SC-2 exists to reject. **The gate was always right; its input was what could not move.** Fixed in the input; `notWatching()` is left at two disjuncts.
  - The startup warning promised the defect in words — *"`watching` will always read yes"* — and now announces the fallback budget instead.
- ⚑ **Two dependencies were `REQUIRED` by the build and declared in no manifest.**
  `nav2_util` and `nav2_ros_common` are both `find_package(... REQUIRED)` (`CMakeLists.txt:20`,
  `:21`) and both linked as exported targets (`:36`, `:37`, `:62`, `:63`), while
  `grep -c nav2_util package.xml` and `grep -c nav2_ros_common package.xml` **both returned 0**.
  `rosdep install --from-paths` installed neither, so the first thing a stranger saw was a
  configure error naming a package this manifest never mentioned. Both are now declared.
- ⚑ **Fifteen stale source line-numbers across four files — every prose citation in the package
  was wrong.** The source moved twice on 2026-09-06 and no citation followed it. Re-measured
  against the committed source and corrected: `ZONE_PARAMETER_FILTER` `:119`/`:123` → **`:198`/
  `:202`** (`README.md` ×2 sites, `CHANGELOG.md`, `package.xml`, `doc/SPEC_COVERAGE.md` ×2);
  the `nominal_defaults` string-array and child reads `:257`/`:260` → **`:337`/`:339`**; the three
  `Failed to lock node` throws `:48`/`:103`/`:172` → **`:78`/`:182`/`:251`**. ⚑ **`:48` had
  drifted onto a doc-comment line and `:257` onto an unrelated `target_node` read** — a citation
  that lands on plausible-looking code is worse than one that lands on nothing, because it
  survives a reader's spot-check.
  - ⚑ **The two pasted compiler transcripts were NOT renumbered, and that is deliberate.**
    Rewriting a line number inside a recorded tool output manufactures a transcript no compiler
    ever emitted. They are labelled instead with the commit that produces them — **hubot
    `f08bb1c`**, where the constant did sit at `:119` — so they are reproducible rather than
    merely disclaimed.
- ⚑ **The one gate written to run WITHOUT a toolchain was reachable only by HAVING the
  toolchain.** `test/safety_invariants_static.sh` states on its own face that it is deliberately
  toolchain-free because the behavioural suite cannot run without ROS and *"the suite did not
  run" reads exactly like "the suite passed."* Its only invocation was `add_test` at
  `CMakeLists.txt:133`, downstream of sixteen `find_package(... REQUIRED)` calls. **Measured
  2026-09-06 on a ROS-free host: cmake aborts at `CMakeLists.txt:14` — the first one, 119 lines
  above the registration — and writes no `CTestTestfile.cmake` at all**, so the gate was never
  registered, not merely skipped. The script also *claimed* `"This gate always runs, everywhere"`;
  it could, and it did not. The false sentence is corrected in place rather than deleted, the
  registration is kept for regression visibility, and the toolchain-free lane
  (`bash test/safety_invariants_static.sh .`) is now named in the README's install-blocker
  section — the one section a reader who cannot build is guaranteed to reach.
  **Verified this turn: 12/12 deterministic green, 6 invariants, no ROS on the host.**
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
  **both** code references to `kMaxPendingSets`, one removing `pending_sets_.clear()`). It is now
  registered with CTest, so it runs in `colcon test`.
  - ⚑ **Corrected 2026-09-06: that read *"the only code reference"* and there are TWO** — the
    guard clause at `src/zone_parameter_filter.cpp:744` and the message at `:749`. The count was
    already 2 at `030bd4d`, the commit that introduced the sentence, so it was **wrong when
    written, not merely stale**. It is corrected rather than deleted because the error is
    load-bearing on the control itself: **a mutant that removes only one of the two leaves INV-C
    GREEN — measured this turn** — so a reader reproducing the recorded control from its own
    description would have got a passing mutant and concluded the gate was blind. A negative
    control is only evidence if its description reproduces it.

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

- ⚑ **This does not build against a released nav2.** `src/zone_parameter_filter.cpp:198`
  needs `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, which is absent from tags `1.5.0` and
  `1.5.1` and present only on branches `main` and `lyrical`. Measured 2026-09-06 against
  tag `1.5.1` (`a6354f3f`) from a clean workspace: compile error. Verified to build and
  test green against upstream `lyrical` HEAD `6f23b11c` only.
- This filter cannot signal to the navigation stack that its output is untrustworthy.
- It has not been run on hardware.
