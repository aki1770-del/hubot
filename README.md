# Hubot

**C++ navigation features for ROS 2 / nav2, shipped out-of-tree.**

Install it, build it, select it in your costmap config. It depends on nav2; it does
not fork it, and it does not need anything merged upstream to reach you.

## Why this exists

`nav2` navigates for a driver that is not a person. The features here start from the
same knowledge and are built so a **human** can rely on what the robot decided —
including when the robot's answer is *"I could not apply this."*

## Feature 1 — `hubot::ZoneParameterFilter`

A costmap filter that applies configured ROS parameter overrides to target nodes
based on the mask value at the robot's pose. It is the out-of-tree sibling of the
upstream `nav2_costmap_2d::ZoneParameterFilter` — same authorship, same Apache-2.0
licence, one behavioural difference that is the reason this package exists.

### The difference: it degrades, it does not abort

Measured on the released `lyrical` branch, 2026-09-05:

- upstream's `checkPendingParameterUpdates()` **throws** `std::runtime_error` when a
  parameter set comes back unsuccessful;
- it is called from the top of `process()`;
- `CostmapFilter::updateCosts()` calls `process()` **bare** — the only `try`/`catch`
  in that file is inside `onInitialize()`;
- `layered_costmap.cpp` contains **no** `try`/`catch` at all.

So the exception leaves the costmap update thread with no handler. **The navigation
node dies on a failed parameter set.**

Upstream's stated intent is right, and this package keeps it verbatim in the source:
a silently-swallowed failure would leave the robot on the value the safety zone tried
to change. But **a robot whose navigation stack aborts is not safer than one that
degrades loudly and keeps navigating.**

Here, every failure is logged at `ERROR` with its reason, the filter latches
`enforcementDegraded()`, and it keeps running. Nothing is swallowed and nothing is
fatal. An integrator can read the flag and decide what their vehicle should do —
which is a decision they could not make when the process was already gone.

## Feature 2 — the human-decision surface

The upstream filter publishes a bare `std_msgs/UInt8` on a topic: a state number.
**A number is a robot's input.** It selects a parameter set, and nothing about it is
a reason. A person deciding whether to rely on the zone needs the basis — which
zone, what changed, on which targets, and above all **whether it actually took
effect.**

Hubot adds `zone_decision`, a `diagnostic_msgs/DiagnosticArray`:

| field | carries |
|---|---|
| `level` | `OK` in force · `WARN` requested but **not yet confirmed** · `ERROR` **when the zone is NOT being enforced** |
| `message` | a sentence a person can act on — *"Zone 2 is NOT being enforced on at least one target. Decide as if the zone's limits are not applied."* |
| `values` | `zone_state`, `mask_state`, `enforced`, `configured`, `unconfirmed_targets`, `degraded_targets`, `pending_parameter_sets`, `targets`, and the triggering `event` |

⚑ **`enforced` is three-valued, and the middle value is the one that matters.**

| value | means |
|---|---|
| `yes` | every target of the current state has **confirmed** |
| `pending` | the sets are **issued and unanswered**. Treat as `NO`. |
| `NO` | a target rejected, threw, or fell silent past `set_parameters_timeout` — **or** the mask named a state that has no configuration |

⚑ **`zone_state` and `mask_state` are different facts.** `zone_state` is the state
whose values are actually in force; `mask_state` is where the mask says the robot
is. They agree in normal operation and **diverge exactly when something is wrong** —
a mask cell naming a state the YAML never declared. One number could never carry
that, which is why both are published.

It is emitted on every zone transition and on every enforcement failure. The
robot's `UInt8` topic is **untouched** — this adds a surface, it does not replace
one. `diagnostic_msgs` was chosen because it is the ROS-native way to address a
person, it needs no new interface package, and existing operator tools already
render it.

**Why this is the point of the package.** Upstream expressed *"I could not enforce
this zone"* by killing the process, which tells a person nothing they can act on.
Feature 1 stops the killing. Feature 2 is what replaces it: the machine says what
it could not do, in terms a human can decide on.

## ⚑ Honest bounds — read these before you rely on it

- ⚑ **SUPERSEDED 2026-09-05. The two bounds here said "not built" and "the
  integration test is owed". Both are now false; the old text is corrected rather
  than deleted, because a reader should see what changed.**
  - **It builds.** `colcon build` green against ROS `lyrical` with `nav2_costmap_2d`
    1.5.1; the plugin library `dlopen`s with 0 undefined symbols. Getting there took
    a 3-file fix: the committed tree did not **configure** (`ament_target_dependencies`
    is gone in this ament era), and behind that sat 276 compile errors — 272 cascading
    from one unqualified base class — plus a real version split: the source was
    `lyrical`, the header was PR-#6372 head.
  - **The integration case named as owed is written.**
    `test/degrade_at_production_caller_test.cpp` runs **upstream's own harness,
    verbatim** (two substitutions: the include and the class) and drives
    `CostmapFilter::updateCosts()` — the caller production uses, which upstream's own
    770-line suite never calls once. Upstream's failing input throws there; here it
    logs at ERROR, latches, and the process lives.
  - **The upstream defect is no longer a static read — it is reproduced.** On vanilla
    released `lyrical`, driven through `updateCosts()`, the process **dies** carrying
    `ZoneParameterFilter: set_parameters failed`, with a negative control that fails.
- **Still true, and load-bearing:** that reproduction is a gtest process, **not a
  robot and not a running `controller_server`**. Nothing here has run on real hardware.
- ⚑ **THE BIGGEST ONE THIS PACKAGE CANNOT DO — read this before you deploy it.**
  **The filter cannot tell the navigation stack that its output is untrustworthy.**
  nav2's channel for that is `Layer::isCurrent()`, which
  `ControllerServer::waitForCostmap()` gates on for `costmap_update_timeout`
  (default 0.30 s) before terminating the goal with `CONTROLLER_TIMED_OUT`. That
  channel is closed to a derived filter three ways, all measured 2026-09-05:
  `CostmapFilter::updateCosts()` runs `setCurrent(true)` **unconditionally after
  `process()` returns**; it is declared **`final`**, so g++ refuses the override
  outright (*"error: virtual function ... overriding final function"*); and
  `Layer::isCurrent()` is **not virtual**. `enforcementDegraded()` is public, and
  **nothing in nav2 calls it.**
  **So an integrator must supply the stop themselves: subscribe to `zone_decision`
  and refuse to drive on `enforced: NO` or `pending`.** A three-line upstream change
  would open the channel (`virtual bool isFilterCurrent()` defaulting to true, used
  as the argument to `setCurrent`); it is written and compile-verified at
  `outputs/cpp/nav2_costmap_filter_isFilterCurrent_2026_09_05.patch`, and it is
  **not submitted**.
- ⚑ **Five defects in this package's OWN honesty, found and fixed 2026-09-05.** Each
  was the success-shaped value this package exists to abolish, inside the package
  that exists to abolish it:
  - `applyState()` **threw** on a mask value no state declared — the same abort
    this package removed from the parameter path, still open on the **mask data**
    path, where a single mis-painted pixel reaches it.
  - Removing that throw was only half the fix, and the worse half: the undeclared
    value was still **recorded as the current state**, so the next transition's
    reset missed and the previous zone's limit rode into a zone that never asked
    for it. Measured: 0.2 m/s where 1.0 was required.
  - `resetFilter()` **cleared the fault flag** on the stated ground that "the
    configuration it referred to is gone". The configuration was never cleared at
    all — no `.clear()` anywhere in the file — while `CostmapFilter::reset()`
    re-runs the config load over it, and `nominal_defaults_` is filled with
    `push_back`. So `ClearEntireCostmap`, which sits in **seven** of nav2's default
    behaviour trees, both **erased the fault while the fault stood** and **doubled
    the nominal-defaults list** every time it ran.
  - `enforced: yes` was published on the **same cycle the sets were issued**, with
    zero confirmations. The code applied its own principle on the way out of a zone
    and violated it on the way in.
  - **A target that never answered was reported as enforced, forever.**
    `kMaxPendingSets` was declared to bound exactly that and referenced zero times —
    and could never have caught it anyway, since one silent set sits at a count of
    one indefinitely. A **deadline** does that job now.
- **A successful set means the target ACCEPTED the value, not that it still holds
  it.** Anything else may set the same parameter afterwards and this filter will not
  notice. UNVERIFIED by construction.
- **The filter only learns what the targets said when the costmap ticks** — the
  result check runs from `process()` and nowhere else, because a costmap filter owns
  no timer. If costmap updates stop, `pending` never resolves and the deadline never
  fires; the last published value simply stands. The `DiagnosticArray` carries a
  header stamp — check it.
- The upstream filter carries other findings raised by nav2's maintainer that are
  **not** addressed here — a `reset()`/`deactivate()` conflation, an event-topic
  ordering gap, a parameter-client discovery race, and a re-apply arming window.
- Nothing here is safety-certified. It is QM-class software.

## Licence

Apache-2.0. Copyright (c) 2026 Komada (aki1770-del) — the same copyright that the
upstream file carries, because it is the same author.
