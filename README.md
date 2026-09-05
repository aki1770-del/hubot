# hubot

**A nav2 costmap filter that applies speed and behaviour limits inside mapped zones —
and tells you, in words, when a zone is not actually being enforced.**

```
find_package(hubot REQUIRED)   # or just add it to your workspace and build
```

Then select `hubot::ZoneParameterFilter` in your costmap plugin list. It depends on
`nav2_costmap_2d`; it does not fork or replace anything you already run.

## The problem it exists for

You paint a zone on a mask and configure what should change inside it — a speed cap
near a doorway, a different footprint in a loading bay. The filter pushes those
values to the nodes that own them.

**Sometimes the push does not land.** A target node is not up yet. It rejects the
value. It accepts and never answers. The mask names a zone number your YAML never
declared.

When that happens your robot is inside a zone whose limits are not applied, and
nothing in a plain state number tells you so. **That is the case this package is
built around.**

## What it does about it

**It keeps navigating, and it says what went wrong.** Every failure is logged at
`ERROR` with its reason, the filter latches `enforcementDegraded()`, and the costmap
keeps ticking. On that path nothing is swallowed and nothing takes the node down.

⚑ **Scoped 2026-09-06; it used to be unqualified.** Three
`throw std::runtime_error{"Failed to lock node"}` remain, at
`src/zone_parameter_filter.cpp:48` (`initializeFilter`), `:103` (`filterInfoCallback`) and
`:172` (`loadStateConfig`). None is on the `process()` / `updateCosts()` path that this
package is about, and the upstream filter carries the same three — but nothing above them
handles an exception either, so "nothing takes the node down" was more than we had shown.

That choice is deliberate and it has a cost, so it is stated plainly: a filter that
carries on has not fixed anything — it has handed you a decision you would not
otherwise have had. **Making that decision possible is the whole design.**

## The surface you read to decide — `zone_decision`

A `diagnostic_msgs/DiagnosticArray`, published on every zone transition and on every
enforcement failure. Your existing operator tools already render it.

| field | carries |
|---|---|
| `level` | `OK` in force · `WARN` requested but not yet confirmed · `ERROR` when the zone is **not** being enforced |
| `message` | a sentence to act on — *"Zone 2 is NOT being enforced on at least one target. Decide as if the zone's limits are not applied."* |
| `values` | `zone_state`, `mask_state`, `enforced`, `configured`, `unconfirmed_targets`, `degraded_targets`, `pending_parameter_sets`, `targets`, and the triggering `event` |

**`enforced` has three values and the middle one is the one people miss.**

| value | means |
|---|---|
| `yes` | every target of the current state has **confirmed** |
| `pending` | the sets are issued and unanswered. **Treat as no.** |
| `NO` | a target rejected, threw, or fell silent past `set_parameters_timeout` — or the mask named a state with no configuration |

**`zone_state` and `mask_state` are different facts.** One is the state whose values
are in force; the other is where the mask says you are. They agree in normal
operation and diverge exactly when something is wrong — a mask cell naming a state
the YAML never declared. A single number cannot carry that, which is why both are
published.

The plain `UInt8` state topic is untouched. This adds a surface; it does not replace
one.

## Read this before you deploy it

**⚑ THIS DOES NOT BUILD AGAINST A RELEASED nav2. Check yours before you plan on it.**

`src/zone_parameter_filter.cpp:119` and `:123` use `nav2_costmap_2d::ZONE_PARAMETER_FILTER`.
That constant is **absent from every nav2 release**. Measured 2026-09-06 by building this
package against upstream tag `1.5.1` (commit `a6354f3f`), from a clean workspace:

```
src/zone_parameter_filter.cpp:119:37: error: 'ZONE_PARAMETER_FILTER' is not a member of 'nav2_costmap_2d'
```

Absent from tag `1.5.0` and tag `1.5.1`; present on branches `main` and `lyrical`. **So today
this package installs only on a nav2 branch HEAD.** Whether anything else also blocks a release
build is **UNVERIFIED** — the compiler stopped at the first error and we did not patch past it.

⚑ **A version number will not tell you which nav2 you have.** Tag `1.5.1` (`a6354f3f`) and
`lyrical` HEAD (`6f23b11c`) both declare `<version>1.5.1</version>`, and this package builds
against exactly one of them. Check for the constant, not the number:

```
grep -r ZONE_PARAMETER_FILTER "$(ros2 pkg prefix nav2_costmap_2d)"/include
```

**⚑ hubot cannot stop your robot, and you must supply that yourself.**

nav2's channel for "this layer's output is untrustworthy" is `Layer::isCurrent()`,
which `ControllerServer::waitForCostmap()` gates on before terminating a goal. That
channel is not open to this filter as shipped. Three facts, re-verified 2026-09-06
against released nav2 **1.5.1** (tag `a6354f3f`): `CostmapFilter::updateCosts()` calls
`setCurrent(true)` unconditionally *after* `process()` returns
(`costmap_filter.cpp:133`); `updateCosts()` is declared `final`
(`costmap_filter.hpp:114`); and `Layer::isCurrent()` is not virtual (`layer.hpp:138`).
`enforcementDegraded()` is public and nothing in nav2 calls it.

⚑ **CORRECTED 2026-09-06. This paragraph said the channel was *"closed to a derived filter
three ways, measured on released `lyrical`"*. Two things were wrong with that.** First,
there is no "released `lyrical`" — `lyrical` is a branch; the releases are tags. Second,
**this package's own header had already retracted the "closed three ways" conclusion**
(`include/hubot/zone_parameter_filter.hpp`, AoU-1) on the ground that
`Layer::setCurrent(bool)` is **public** (`layer.hpp:147`, inside the `public:` region that
opens at `:61` and ends at `:181`, verified 2026-09-06) — so the capability exists in the
base class and is erased by *statement order* in the derived one, not by an architecture.
The retracted sentence was restated here in the same commit that corrected the header.
**What you must do is unchanged either way**, so the instruction below still stands.

**So: subscribe to `zone_decision` and refuse to drive on `enforced: NO` or
`pending`.** If you do not, hubot has told you and nothing has listened.

**It has never run on a robot.** Verified 2026-09-06 from a clean workspace against
**upstream `lyrical` branch HEAD `6f23b11c`, with no local nav2 patches on the path**:
`colcon build` green, `ldd -r` reports zero undefined symbols, `pluginlib` resolves
`hubot::ZoneParameterFilter` out of the ament index, and 21 of 21 tests pass — including a
live `LayeredCostmap::updateMap()` and a **negative control that requires upstream's own
filter, loaded from that same build, to throw out of `updateMap()` on the same input**. It
does (`"ZoneParameterFilter: set_parameters failed: parameter 'readonly_speed' cannot be set
because it is read-only"`), so the harness can see the failure it rules out.

⚑ **CORRECTED 2026-09-06 — this bound previously read *"builds green against ROS `lyrical`
with `nav2_costmap_2d` 1.5.1"*. The tree it was green against declared `<version>1.5.0</version>`
and carried local nav2 modifications nothing warned about: measured 2026-09-06 against upstream
`lyrical` HEAD, **six modified files** — `layered_costmap.hpp`/`.cpp` and
`footprint_subscriber.hpp`/`.cpp` (26 lines), `keepout_filter.cpp` (7) and
`nav2_util/src/path_utils.cpp` (30) — **two of them the production caller's own class**, in a
directory that is not a git repository. And against *released* 1.5.1 it does not build at all (see the first bound above). A build inside a tree that may hold your own
edits says nothing about a stranger's build.**

That is a gtest process, not a vehicle. Nothing here has run on real hardware, and nothing here
has run inside a real `controller_server`.

**A confirmed set means the target accepted the value, not that it still holds it.**
Anything else may set the same parameter afterwards and this filter will not notice.
Unverified by construction.

**It only learns what your targets said when the costmap ticks.** The result check
runs from `process()` and nowhere else, because a costmap filter owns no timer. If
costmap updates stop, `pending` never resolves, the deadline never fires, and the
last published value simply stands. The `DiagnosticArray` carries a header stamp —
check it.

**It is QM-class software. Nothing here is safety-certified.**

## Lineage and licence

hubot's filter derives from `nav2_costmap_2d::ZoneParameterFilter` and is written by
the same author, with the same Apache-2.0 licence and the same copyright line:
**Copyright (c) 2026 Komada (aki1770-del)**. The upstream filter's design intent —
that a failed parameter set must never be silently swallowed, because that leaves a
robot on the value a safety zone tried to change — is correct, is kept here
verbatim in the source, and is the reason the reporting surface exists at all.

Findings raised on the upstream filter by nav2's maintainer are **not** addressed in
this package: a `reset()`/`deactivate()` conflation, an event-topic ordering gap, a
parameter-client discovery race, and a re-apply arming window. If you run the
upstream filter, those are its business and not superseded by anything here.
