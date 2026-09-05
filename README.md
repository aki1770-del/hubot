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
| `level` | `OK` in force · `ERROR` **when the zone is NOT being enforced** |
| `message` | a sentence a person can act on — *"Zone 2 is NOT being enforced on at least one target. Decide as if the zone's limits are not applied."* |
| `values` | `zone_state`, `enforced`, `configured`, `pending_parameter_sets`, `targets`, and the triggering `event` |

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
- ⚑ **Two defects in this package's OWN honesty, found and fixed 2026-09-05:**
  `resetFilter()` did not clear `enforcementDegraded()` although the header stated a
  reload clears it — so the owed test would have **failed against the shipped
  contract**; and the latch was cleared on leaving the mask **before the restore was
  confirmed**, publishing `enforced: yes` on a restore nothing had confirmed. Both are
  the success-shaped value this package exists to abolish, inside the package that
  exists to abolish it.
- The upstream filter carries other findings raised by nav2's maintainer that are
  **not** addressed here — a `reset()`/`deactivate()` conflation, an event-topic
  ordering gap, a parameter-client discovery race, and a re-apply arming window.
  This release fixes the fatal path only.
- Nothing here is safety-certified. It is QM-class software.

## Licence

Apache-2.0. Copyright (c) 2026 Komada (aki1770-del) — the same copyright that the
upstream file carries, because it is the same author.
