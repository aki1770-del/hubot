# hubot

## Mission

> **hubot is a method to bridge a robot and a human, from the perspective of the move.**
>
> **A move is mobility: something starts at A and arrives at B. Between A and B there is
> always an obstacle. To avoid collision and to make the journey a better one, hubot
> offers the human a suggestion before the actual risk arrives.**

Four things follow from that sentence, and every component here is built to them.

**The unit is one move, A to B.** Not a feature, not a topic, not a package. If a thing
we build does not serve a move that is underway, it does not belong here.

**The obstacle is the normal case.** Something is always between A and B — a doorway, a
bay, a person walking through, a limit that has to hold. A component whose value only
shows up in a rare failure has mis-read the problem.

**The far end of the bridge is a person.** A state number, a cost value, a boolean that
another node consumes — that is machinery. Useful, necessary, and not the bridge. **The
bridge ends in someone who can read it.**

⚑ **Before the risk, and as an offer.** A report issued after the collision is a log.
hubot earns its place in the window where the risk has **not yet arrived** and a person
can still act — and what it puts there is a **suggestion**, never a command. **The person
keeps the decision.** A component that seizes the decision has replaced the human it was
built to serve.

---

The first component is a nav2 costmap filter. It applies speed and behaviour limits
inside mapped zones, and — the part that matters — **it tells a person, in words, while
a limit is still only *requested* and not yet in force.**

```
find_package(hubot REQUIRED)   # or just add it to your workspace and build
```

Then select `hubot::ZoneParameterFilter` in your costmap plugin list. It depends on
`nav2_costmap_2d`; it does not fork or replace anything you already run.

## How zones work, if you have not used a costmap filter before

Skip to **What it does about it** if you already run keepout or speed filters — this
is the same mechanism.

### A zone is a colour on a picture of your map

A costmap filter reads a second image laid over your map, called a **mask**. It is an
ordinary occupancy-grid image with the same resolution and origin as your map, and
you edit it in any image editor. What matters is the **value of each pixel**:

```
0    outside every zone — normal operation
1    zone 1
2    zone 2
…    up to 255
```

Paint the doorway area `1`, paint the loading bay `2`, leave everything else `0`. That
is the whole map side of it. The mask is published by nav2's standard
`costmap_filter_info_server` and `map_server` pair, exactly as it is for a keepout
filter; nothing new is involved.

### You say what each number means

Each number is a **state**, and you describe a state by listing the parameters it
should change and the value it should set. In your costmap YAML:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      filters: ["zone_filter"]

      zone_filter:
        plugin: "hubot::ZoneParameterFilter"
        filter_info_topic: "/costmap_filter_info"

        # every state you use, by name
        states: ["slow_doorway", "loading_bay"]

        slow_doorway:
          id: 1                       # the pixel value you painted
          setpoints: ["cap_speed"]
          cap_speed:
            node: "/controller_server"
            parameter: "FollowPath.max_vel_x"
            value: 0.3

        loading_bay:
          id: 2
          setpoints: ["cap_speed", "widen_footprint"]
          cap_speed:
            node: "/controller_server"
            parameter: "FollowPath.max_vel_x"
            value: 0.15
          widen_footprint:
            node: "/local_costmap/local_costmap"
            parameter: "robot_radius"
            value: 0.55

        # what to restore when the robot leaves every zone (pixel value 0)
        # ⚑ SEE THE WARNING BELOW — this block does not work in nested YAML.
        nominal_defaults: ["normal_speed", "normal_footprint"]
        nominal_defaults.normal_speed:
          node: "/controller_server"
          parameter: "FollowPath.max_vel_x"
          value: 1.0
        nominal_defaults.normal_footprint:
          node: "/local_costmap/local_costmap"
          parameter: "robot_radius"
          value: 0.35
```

`id` is the pixel value. `1` to `255` are yours; **`0` is reserved** and means *leave
every zone* — that is when `nominal_defaults` is restored.

> ### ⚑ Two warnings about this example, both measured 2026-09-06
>
> **1. `nominal_defaults` cannot be written as nested YAML, and we do not yet have a
> form that is proven to load.** The filter reads `<filter>.nominal_defaults` as a
> **string array** (`src/zone_parameter_filter.cpp:257`) and also reads
> `<filter>.nominal_defaults.<name>.node` as children of that same key (`:260`). In a
> ROS 2 parameter file one key cannot be both a sequence and a mapping. The dotted
> form printed above is written the only way the two can coexist in one document, and
> **whether the parameter loader accepts it is untested.** Until it is, set those
> entries from a launch file or the command line, where the dotted names are exactly
> what the code reads. `states` does **not** have this problem — the state names are
> different keys from `states` itself.
>
> **2. No YAML file has ever configured this filter.** The package contains **zero**
> `.yaml` files and **zero** tests that load parameters from one; every test sets the
> parameters programmatically. So the configuration path *you* would use is the one
> path this package has never exercised. The parameter **names** above are read
> straight from the source and are correct. The **file form** is not yet verified.

### What actually happens when the robot drives in

On each costmap update the filter reads the mask pixel under the robot's pose. When
that value changes, it calls **`set_parameters` on the nodes you named**.

⚑ **This is the part worth understanding, because everything else on this page follows
from it.** `max_vel_x` does not belong to the costmap. It belongs to
`controller_server`, **a different process**. The filter does not set a variable — it
sends a request across ROS to another node and waits for that node to answer.

So driving into a doorway does this:

```
mask pixel under the robot goes 0 → 1
  → set_parameters(/controller_server, FollowPath.max_vel_x = 0.3)
  → /controller_server replies: accepted
  → the robot slows down
```

And driving out reverses it, restoring `nominal_defaults`.

### And sometimes the reply does not come

That request is a call to another process, and calls to other processes fail:

- `/controller_server` **has not started yet**, so nothing is listening;
- it **rejects** the value — the parameter is read-only, or out of its declared range;
- it **accepts and never answers**, because it is wedged or the message was dropped;
- the mask names a state your YAML **never declared** — you painted `3` and configured
  only `1` and `2`.

In every one of those cases **the robot is inside the doorway and still doing a metre a
second.** The zone is painted, the configuration is right, and the limit is not on.

Nothing in a plain state number tells you that. A number says *where the robot is*; it
does not say *whether what you asked for actually happened.* **That gap is what this
package exists for.**

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

**`enforced` has three values, and ⚑ the middle one is the reason this package exists.**

| value | means |
|---|---|
| `yes` | every target of the current state has **confirmed** |
| `pending` | ⚑ **the sets are issued and unanswered.** Nothing has gone wrong yet — and nothing is holding the robot back either. **Treat as no.** |
| `NO` | a target rejected, threw, or fell silent past `set_parameters_timeout` — or the mask named a state with no configuration |

⚑ **`pending` is the window this whole package is built for.** It is the moment the
robot is entering a zone, the limit has been asked for, and **nobody yet knows whether
it took.** The collision has not happened. The speed cap may still land a hundred
milliseconds from now. **A person reading `pending` is being told something before the
risk, not after it** — which is the only kind of telling that leaves them anything to
do.

`yes` and `NO` are history. **`pending` is the suggestion.**

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

**⚑ hubot does not stop your robot. You hold the stop, and that is deliberate.**

**This is design, not shortfall** — hubot offers a person a suggestion; it does not take
the decision. But it does mean **the stop has to exist on your side, and if you do not
build it, nothing acts on what hubot says.** The mechanism, so you can see exactly what
is and is not available to you:

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
