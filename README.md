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

---

## ⚑ YOU CANNOT INSTALL THIS TODAY. Read this before the instructions below.

**No released `nav2_costmap_2d` can build this package.** `src/zone_parameter_filter.cpp:119`
and `:123` name `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, a `uint8_t` constant carried on the
`main` and `lyrical` **branches** and in **no release**. Against tag `1.5.1`, from a clean
workspace, the compiler says so on the first file:

```
src/zone_parameter_filter.cpp:119:37: error: 'ZONE_PARAMETER_FILTER' is not a member of 'nav2_costmap_2d'
```

**Do not take our word for it — check your own installation:**

```
grep -r ZONE_PARAMETER_FILTER "$(ros2 pkg prefix nav2_costmap_2d)"/include
```

**No output means this package will not build for you.** A version number cannot tell you:
tag `1.5.1` and `lyrical` HEAD both declare `<version>1.5.1</version>`, and only the second
one works.

**What you are waiting on:** a nav2 release that carries that constant. We have not measured
one existing. **That command is your clock** — when it prints a line, this package builds for
you. It is a wait, not a wall, and it is checkable on your machine rather than datable on this
page.

⚑ **And it is our line, not upstream's omission.** That constant is a discriminator for the
`CostmapFilterInfo.type` field — a number your own `costmap_filter_info_server` reads out of
your YAML. This package needs to **agree** with that number; it never needed to **obtain** it
from nav2's header. The dependency is a copying mistake of ours, the fix is ours, and it is
not made yet.

**Everything below describes a package you cannot yet install.** It is accurate about what the
filter does and how it is configured; it is aspirational about your getting it.

---

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
| `level` | `OK` in force · `WARN` requested but not yet confirmed · `ERROR` when the zone is **not** being enforced · `STALE` when the filter is **not watching** |
| `message` | a sentence to act on — *"Zone 2 is NOT being enforced on at least one target. Decide as if the zone's limits are not applied."* |
| `values` | `zone_state`, `mask_state`, `enforced`, `configured`, `unconfirmed_targets`, `degraded_targets`, `pending_parameter_sets`, `targets`, the triggering `event`, and the liveness fields `watching`, `costmap_age_s`, `report_seq`, `report_period_s`, `valid_for_s` |

**`report_period_s`** is how often the next message is due — the heartbeat period actually in
effect, not the configured default. **`valid_for_s`** is how long *this* message should be
treated as current: `2.5 x report_period_s`. It is wide enough to survive one dropped or late
heartbeat without expiring a healthy publisher, and narrow enough that a dead one is stale
inside three periods. **If `now - header.stamp > valid_for_s`, stop believing the message** —
including its `enforced` field. That is the check you have to perform; see the bound at the end
of the liveness section for why we cannot perform it for you.

**It is published on every zone transition, on every enforcement change, and — ⚑ since
2026-09-06 — every `liveness_period` seconds whether or not anything changed.** That last
one is the subject of *"Silence means I am not watching"* below, and it is the reason a
quiet `zone_decision` is no longer the same thing as a healthy one.

**`enforced` has four values, and ⚑ the second one is the reason this package exists.**

| value | means |
|---|---|
| `yes` | every target of the current state has **confirmed** — ⚑ and the filter is still watching |
| `pending` | ⚑ **the sets are issued and unanswered.** Nothing has gone wrong yet — and nothing is holding the robot back either. **Treat as no.** |
| `unknown` | ⚑ **nobody is watching.** Either the costmap has stopped calling the filter, **or it has not called it yet** — since 2026-09-06 both read the same, because to you they are the same fact. Nothing has failed; nothing is being checked either. See `watching` below |
| `NO` | a target rejected, threw, or fell silent past `set_parameters_timeout` — or the mask named a state with no configuration |

⚑ **`unknown` was added 2026-09-06 and it is a breaking change to this vocabulary.** It is
made now, deliberately, because this package has no remotes and no consumer can be holding
the old set — the cheapest moment it will ever be. **The rule that goes with it is a
whitelist, and the old blacklist below has been corrected for the same reason.**

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

**So: subscribe to `zone_decision` and ⚑ PROCEED ONLY ON `enforced: yes`.** If you do not,
hubot has told you and nothing has listened.

⚑ **CORRECTED 2026-09-06, and the correction is about shape, not wording.** This read
*"refuse to drive on `enforced: NO` or `pending`"* — **a blacklist**, which cannot be
complete, and which **fails open**: a consumer coded literally against those two strings
would have read the new `unknown` as permission to drive. A rule that admits every value
it has not heard of is the wrong rule on a surface like this one regardless of how many
values exist today. It is a whitelist now.

**It has never run on a robot.** Verified 2026-09-06 from a clean workspace against
**upstream `lyrical` branch HEAD `6f23b11c`, with no local nav2 patches on the path**:
`colcon build` green, `ldd -r` reports zero undefined symbols, `pluginlib` resolves
`hubot::ZoneParameterFilter` out of the ament index, and **28 of 28 tests pass** (21 before
2026-09-06; `colcon test-result --all`) — including a
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

⚑ **THIS BOUND SAID THE FOLLOWING UNTIL 2026-09-06, AND IT IS KEPT BECAUSE IT IS WHY THE
SECTION BELOW EXISTS:** *"It only learns what your targets said when the costmap ticks. The
result check runs from `process()` and nowhere else, because a costmap filter owns no timer.
If costmap updates stop, `pending` never resolves, the deadline never fires, and the last
published value simply stands. The `DiagnosticArray` carries a header stamp — check it."*

**Every clause of that was true of the code, and the countermeasure was the last two words,
addressed to you.** See *"Silence means I am not watching"*. What is left of it is stated
there, honestly and in full: **if the whole node dies, the last message still stands.**

**It is QM-class software. Nothing here is safety-certified.**

## ⚑ Silence means "I am not watching", not "you are safe"

A costmap filter is only called when the costmap ticks. So if the costmap stopped, this
filter stopped — and a topic that only speaks when something changes says exactly nothing
in that case. **An `OK` from ninety seconds ago renders identically to an `OK` from now.**
You would read *the zone is enforced*. The truth would be *nobody is checking*.

**That is the failure this package was written to abolish, and it was inside the package.**

**What it does now.** A liveness timer on the node — **not** on the costmap update loop:

1. **It publishes every `liveness_period` (default 1.0 s) whether or not anything changed.**
   Presence of the message is the claim *"I am watching."* Absence of it is the claim
   *"I am not."* Publishing only on change is what made a healthy quiet filter and a dead
   one produce the same observable, and you cannot act on an observable that is identical in
   the good case and the bad one.
2. **After `costmap_silence_timeout` (default 2.0 s) without a call, it says so** —
   `watching: NO`, level **`STALE`**, and `enforced` becomes **`unknown`**. `STALE` rather
   than `ERROR` on purpose: nothing was measured and found bad; nothing was measured.
   ⚑ **And since 2026-09-06 the same three values are reported BEFORE the first call too.**
   `initializeFilter()` runs at configure and starts this heartbeat; the costmap's update
   thread does not exist until activate. In between, the filter had been publishing level
   `OK` / `enforced: yes` / `watching: yes` having read no mask, applied no state and
   confirmed nothing — indistinguishable from a zone enforced on every target. **A routine
   `ClearEntireCostmap` re-opened that same window every time it fired.** The gate now lives
   in the value (`enforcementToken()` and `watching` both derive from one predicate) rather
   than at each caller, because the three previous fixes for this same defect all went into
   callers and every new caller then arrived without them.
3. ⚑ **It also does the work.** The `set_parameters` deadline check needs the clock and the
   futures and nothing from the costmap, so the timer runs it. **`pending` now resolves and
   the deadline now fires with the costmap stopped** — which is the larger half of the old
   bound above going false.
4. **It covers a second door the old bound never named.** `CostmapFilter::updateCosts()`
   returns early when `enabled_` is false and never calls the filter
   (`costmap_filter.cpp:128-130`), and `enabled_` is flipped by the base class's own
   `<name>/toggle_filter` service (`:82-86`). **A disabled filter used to look exactly like
   an enforced zone.** Now it reports `watching: NO`.

**Why a timer is possible at all**, since "a costmap filter owns no timer" was our own
sentence: `Layer::node_` is a live handle in the protected region (`layer.hpp:186`), and
**the node is spun on a different thread from the costmap update loop** — `Costmap2DROS`
always creates `map_update_thread_` in `activate()` (`costmap_2d_ros.cpp:314`), while its
node is spun separately by whoever owns it (`controller_server.cpp:72`,
`planner_server.cpp:78`, and `rclcpp::spin` in `costmap_2d_node.cpp:47`). **All three
non-test construction sites in nav2 have that property, and it is structural in
`Costmap2DROS` rather than a habit of any one server.** So the two threads fail
independently, and the timer keeps running through exactly the failure that stops the
filter.

| parameter | default | set `<= 0` to |
|---|---|---|
| `liveness_period` | `1.0` s | disable the heartbeat entirely (warned at startup) |
| `costmap_silence_timeout` | `2.0` s | keep the heartbeat but never report a stopped costmap |

⚑ **THE PART THIS DOES NOT CLOSE, AND CANNOT.** **If the node itself dies, the timer dies
with it**, no message arrives, and the last one stands — exactly as before. **Nothing
running inside a process can announce that process's own death.** What you get instead of a
bare stamp is a **declared expiry**: every message carries `report_period_s` and
`valid_for_s`, so three lines of consumer code can decide the report has expired, and
`report_seq` advances on every publish so a frozen `/clock` cannot fake liveness. **That is
strictly better than "check it". It is still something you must check.**

**The complete answer lives in your process, not ours, and we have not built it:** a
`DEADLINE` QoS on your subscription, or a `diagnostic_aggregator` staleness rule, raises the
alarm in *your* node when ours goes quiet. ⚑ **NOT IMPLEMENTED HERE AND NOT MEASURED.**
Naming it is not the same as having built it, and it is written here as a gap rather than
offered as a feature.

## Lineage and licence

hubot's filter derives from `nav2_costmap_2d::ZoneParameterFilter` and is written by
the same author, with the same Apache-2.0 licence and the same copyright line:
**Copyright (c) 2026 Komada (aki1770-del)**. The upstream filter's design intent —
that a failed parameter set must never be silently swallowed, because that leaves a
robot on the value a safety zone tried to change — is correct, is kept here
verbatim in the source, and is the reason the reporting surface exists at all.

Findings raised on the upstream filter by nav2's maintainer are **not fixed upstream** by
anything in this package — if you run the upstream filter, those remain its business.

⚑ **But three of them describe a design this package SHARES, and saying otherwise was wrong.**
Until 2026-09-06 this section implied they were simply not our subject. They were checked one by
one against this source instead, and the result is a table in `doc/SPEC_COVERAGE.md` §4 giving a
verdict and its evidence for each. In short: the `reset()`/`deactivate()` conflation **is here**
and cannot be fixed the way he prescribed, because `CostmapFilter::reset()` is `final`; the
event-topic ordering gap **was here and is fixed**; the parameter-client rebuild **is here**,
with its discovery-race consequence **UNVERIFIED, not cleared**; and the re-apply arming window
**cannot occur here** because there is no re-apply mechanism at all. Each of those verdicts has a
grep behind it, and the ones that occur have a test that fails without the fix.
