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

## ⚑ Honest bounds — read these before you rely on it

- **This package has not been built or tested yet.** No `colcon build` has run
  against it, and the unit test in `test/` deliberately does **not** claim to prove
  the abort is gone: the case that matters is an integration case with a live
  executor and a rejecting target node, and it is **owed, not written**. A suite that
  cannot fail on the real defect has measured nothing.
- **The upstream defect above is a static call-chain read**, not an observed crash.
  It has not been reproduced on a running robot.
- The upstream filter carries other findings raised by nav2's maintainer that are
  **not** addressed here — a `reset()`/`deactivate()` conflation, an event-topic
  ordering gap, a parameter-client discovery race, and a re-apply arming window.
  This release fixes the fatal path only.
- Nothing here is safety-certified. It is QM-class software.

## Licence

Apache-2.0. Copyright (c) 2026 Komada (aki1770-del) — the same copyright that the
upstream file carries, because it is the same author.
