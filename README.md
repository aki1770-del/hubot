# hubot

**A nav2 costmap filter.** It applies speed and behaviour limits inside mapped zones, and
— the part that matters — **it tells a person, in words, when a limit was asked for and
you do not yet know whether it took.**

<!-- OBTAIN-LINE: live. This block is the clone command; keep it first on the page. -->

```
git clone https://github.com/aki1770-del/hubot
```

⚑ **Read the next section before you build it.** The package builds and passes against a released
`nav2_costmap_2d` in a clean room — **and it has never run on a robot.** The current release is `0.1.1`.
⚑ **If you pinned `0.1.0`, upgrade.** That tag names a tree twelve commits older, and it carries
**one silent breaking change** — the namespace join recorded as PI-16 — which its own `CHANGELOG.md`
does **not** mention, because the disclosure was written after the tag was cut. That tree also
contains no CI workflow at all, so no gate has ever run on it. `0.1.1` is the first release this
project's gate has passed.

⚑ **Which ROS 2 distribution — read this before you clone.** This package requires **nav2 >= 1.5.0**,
which today means **`lyrical` on Ubuntu 26.04 (`resolute`)**. On **`jazzy`** (nav2 `1.3.12`) and
**`kilted`** (nav2 `1.4.2`) it **does not build** — and it fails at *configure*, in about a second,
rather than part-way through a compile. Measured 2026-09-07 against both distributions' real
published packages, not inferred from headers: `find_package` fails at `CMakeLists.txt:21` because
**`nav2_ros_common` does not exist before nav2 1.5.0**, and no `ros-jazzy-nav2-ros-common` or
`ros-kilted-nav2-ros-common` is published at all.

<!-- BEGIN GENERATED distro-floor — edit .github/supported_distros.yml, then run scripts/distro_floor.py --render -->

<!-- Generated. Every row is proved on every run by the `distro floor` CI job,
     in both directions: a `no` that starts building reddens the job too. -->

| ROS 2 | Ubuntu | nav2 | builds today |
|---|---|---|---|
| `lyrical` | resolute | `1.5.1` | **yes** |
| `kilted` | noble | `1.4.2` | no — missing `nav2_ros_common` |
| `jazzy` | noble | `1.3.12` | no — missing `nav2_ros_common` |

Measured against the live package feed, not inferred from headers.

<!-- END GENERATED distro-floor -->

A version shim would not be enough, and the reason is worth stating rather than leaving you to
discover it: **`CostmapFilter::process()` is a pure virtual whose signature changed at 1.5.0** —
`geometry_msgs::msg::Pose2D` before it, `geometry_msgs::msg::Pose` after. One `process()` cannot
override both, so on an older nav2 this class is **abstract** and pluginlib cannot instantiate it
even if every other difference were papered over. If you are on `jazzy` or `kilted`, this package
has nothing for you today, and we would rather you learn that here than from a build log.

**Three more bounds, before the pitch rather than after it:**

- ⚑ **It has never run on a robot.** It builds, its plugin loads, its tests drive it
  through a live `LayeredCostmap` in a gtest process, and — since 2026-09-06 — a shipped
  launch file brings it up inside nav2's own `Costmap2DROS` in a container. Not a
  vehicle. **Pass counts differ by which nav2 you build against, so they are not quoted
  here without one**; both are under *"Read this before you deploy it"*.
- **It is QM-class software. Nothing here is safety-certified.**
- **It does not stop your robot.** It reports; you hold the stop. If you do not build the
  stop, nothing acts on what hubot says.

---

## Run it

⚑ **Until 2026-09-06 this package shipped a plugin and told you to build the other four
things a costmap filter needs before it can run at all** — a mask, a `map_server`
publishing it, a `costmap_filter_info_server`, and params plus a launch file wiring them
together. That homework is now in the package.

```bash
colcon build --packages-select hubot
ros2 launch hubot zone_filter_demo_launch.py
```

In another terminal:

```bash
ros2 topic echo /zone_decision
```

You should see `enforced: yes` and *"Zone 1 is in force."*, and the target process should
log `set_parameters ARRIVED HERE: demo_speed -> 0.3`. **If you see anything else, this
launch file has not done its job — say so.**

**What it starts**, all of it either a stock nav2 executable or a file this package
installs — nothing is read from a source tree:

| | |
|---|---|
| `maps/zone_mask.pgm` + `.yaml` | the painted zone. **Plain text — open it.** Its own header carries the two things about masks that are easy to get wrong. |
| `maps/demo_map.pgm` + `.yaml` | the world, so the costmap's `static_layer` declares a window. Without a bounds-declaring plugin the filter is driven **zero times** and everything still looks healthy. |
| `nav2_map_server` ×2 | one publishes the map, one publishes the mask. |
| `costmap_filter_info_server` | tells the filter where the mask is, and with what `type`. |
| `nav2_costmap_2d` | nav2's own standalone costmap, in namespace `/local_costmap` — no `controller_server`, no planner, no behaviour tree. |
| `zone_target_demo_node` | **the node the limit is applied to.** On your robot this is `controller_server` and `FollowPath.max_vel_x`; the demo ships one so the first run ends in a working stack instead of in writing one. |

### Two controls ship with it, and running one is worth more than running the demo twice

```bash
ros2 launch hubot zone_filter_demo_launch.py overlay:=overlay_target_readonly.yaml
ros2 launch hubot zone_filter_demo_launch.py overlay:=overlay_target_inside_namespace.yaml
```

Both are **supposed** to end in `enforced: NO`, for two different reasons, with two
different sentences — one target refuses the set, the other never receives it. A value
with no control beside it is not evidence, which is this package's whole argument, so its
own demo carries the controls. Each file says what to look for.

You can also move the robot without a simulator:

```bash
ros2 launch hubot zone_filter_demo_launch.py robot_x:=2.0 robot_y:=2.0   # outside the zone
```

⚑ **The robot does not move during a run.** Nothing here is a simulator, and this package
still has not run on one.

---

## Mission

> **hubot is a method to bridge a robot and a human, from the perspective of the move.**
>
> **A move is mobility: something starts at A and arrives at B. Between A and B there is
> always an obstacle. To avoid collision and to make the journey a better one, hubot
> offers the human a suggestion before the risk arrives.**

⚑ **And the last clause of that sentence is a destination, not a description — so read it
with what is built.** Today this component speaks when a zone is entered, when enforcement
fails, and once every heartbeat period in between. **No lookahead is built.** It is handed
the robot's *current* pose and nothing else — `process()` takes one
`geometry_msgs::msg::Pose` (`src/zone_parameter_filter.cpp:525-528`), and there is no plan,
path, trajectory or velocity anywhere in this component.

**The *before* it earns today is before the ENFORCEMENT OUTCOME is known** — the window in
which the limit has been asked for and nobody yet knows whether it took. That window is
real, it is measured, and it is the reason this package exists; it is called `pending` and
it is described below. **The *before* the mission sentence names — before the HAZARD — is
the next thing, and it is not here yet.**

Three things follow from the mission and are true of this component today; the fourth is
what the sentence is still reaching for.

**The unit is one move, A to B.** Not a feature, not a topic, not a package. If a thing
we build does not serve a move that is underway, it does not belong here.

**The obstacle is the normal case.** Something is always between A and B — a doorway, a
bay, a person walking through, a limit that has to hold. A component whose value only
shows up in a rare failure has mis-read the problem.

**The far end of the bridge is a person.** A state number, a cost value, a boolean that
another node consumes — that is machinery. Useful, necessary, and not the bridge. **The
bridge ends in someone who can read it.**

⚑ **As an offer, and — one day — ahead of the risk.** What hubot puts in front of a person
is a **suggestion**, never a command; **the person keeps the decision**, and a component
that seizes it has replaced the human it was built to serve. That half is built. The other
half — arriving *ahead of the hazard* rather than ahead of the answer — is not, and this
page will not claim it until something in this repository does it.

---

## ⚑ IT BUILDS AGAINST A RELEASE NOW. It could not this morning — read this before the instructions below.

**Until 2026-09-06 no released `nav2_costmap_2d` could build this package, and this section said
so in those words.** `src/zone_parameter_filter.cpp` named `nav2_costmap_2d::ZONE_PARAMETER_FILTER`,
a `uint8_t` constant carried on the `main` and `lyrical` **branches** and in **no release**.
Against tag `1.5.1`, from a clean workspace, the compiler said so on the first file:

```
src/zone_parameter_filter.cpp:119:37: error: 'ZONE_PARAMETER_FILTER' is not a member of 'nav2_costmap_2d'
```

*(Verbatim compiler transcript, kept unedited, from hubot `f08bb1c` against tag `1.5.1`; the
line number is that commit's. It stays on this page because it is the record of what this
package was, and a correction that erases its own evidence reads as if the gap had never been
there.)*

**What changed.** That constant was our copying mistake, not upstream's omission. It is a
discriminator for the `CostmapFilterInfo.type` field — a number your own
`costmap_filter_info_server` publishes out of your YAML. This package needs to **agree** with
that number; it never needed to **obtain** it from nav2's header. It is now
`hubot::kZoneParameterFilterType` (`include/hubot/zone_parameter_filter.hpp:64`), and two
`static_assert`s guard it: one fires on **any** nav2 if upstream renumbers its filters, one fires
wherever nav2 carries the constant and disagrees with ours. **Measured**: the full nav2
dependency chain built from tag `1.5.1`; this package built and installed against it; the plugin
resolved and ran through a live `LayeredCostmap` there. **Building is not passing**: 23 of 26
tests pass on that release — the three that do not are one pre-existing case identical on
branch, and two whose *upstream-comparison* arm needs a plugin that exists only on branch.
`1.5.0` was not built and is not claimed.

**Do not take our word for it — check your own installation:**

```
grep -r ZONE_PARAMETER_FILTER "$(ros2 pkg prefix nav2_costmap_2d)"/include
```

**That command used to be your clock. It is now a diagnostic.** No output means you have a
release-era nav2 — **this package builds for you anyway**, on its own discriminator. Output means
your nav2 carries the constant, and the build cross-checks ours against it at compile time.
Either way a version number cannot tell you which you have: tag `1.5.1` and `lyrical` HEAD both
declare `<version>1.5.1</version>`.

**What still stands between you and running it is not the build, and it is no longer the obtain
line either.** This repository is public and the clone command at the top of this page works.
*(⚑ This paragraph read "this repository has no remote ... the obtain line is a placeholder"
until 2026-09-07. That was true when it was written on 2026-09-06, and the remote landed without
the sentence being corrected. Recorded rather than quietly deleted: a page that describes its own
state goes stale the moment someone changes that state.)* What remains is that **nothing here has
run on a robot**, and the launch bring-up has been exercised only inside a container.

**Everything below describes a package you can now build.** It is accurate about what the filter
does and how it is configured.

### One thing you CAN run today, with no toolchain at all

The behavioural suite needs colcon, nav2, rclcpp and a live executor, so on the host you are
almost certainly reading this from it cannot run — and **"the suite did not run" reads exactly
like "the suite passed."** The static safety gate is written for precisely that host. It is
bash, grep and awk, it needs nothing installed, and it finishes in under a second:

```
bash test/safety_invariants_static.sh .
```

Six invariants over the committed source; exit `0` if all hold, `1` if any is violated, and
each failure line names the file position that refutes it. **It proves the shape of the source,
never the behaviour of the robot** — it is a sound partial, not a substitute for the suite.

⚑ *This line exists because the gate was reachable only through `colcon test`, where it is
registered at `CMakeLists.txt:133` — downstream of sixteen `find_package(... REQUIRED)` calls.
Measured 2026-09-06 on a ROS-free host: `cmake` aborts at `CMakeLists.txt:14`, the very first
one, 119 lines before the registration, and writes no `CTestTestfile.cmake` at all. **The one
gate built to run without a toolchain was reachable only by having the toolchain**, and nothing
on any surface told you the command above existed.*

---

```
find_package(hubot REQUIRED)   # or just add it to your workspace and build
```

Then select `hubot::ZoneParameterFilter` in your costmap plugin list. It depends on
`nav2_costmap_2d`; it does not fork or replace anything you already run.

## What proves it, and where to look

**A gate runs on every push and every pull request** — `.github/workflows/gate.yml`, four jobs:

| job | what it decides |
|---|---|
| `suite (released nav2)` | the package builds and its ten CTest targets pass against a **released** `nav2_costmap_2d` installed from `packages.ros.org`, with dependencies resolved from `package.xml` rather than from a list someone maintains by hand |
| `launch falsifier (bring-up)` | the shipped launch file actually brings the filter up out of the **install space**, across four cases |
| `live stack (harness, released nav2)` | the harness in `hubot_live_stack/` stands up nav2's own `controller_server` and a real `Costmap2DROS` **out of process**, and drives three conditions |
| `negative controls (prose + tree)` | the claims on this page are re-derived from the tree, and every check is proven able to fail |

⚑ **The gate has been watched failing on purpose.** A branch carrying a deliberately broken safety
invariant reddened **only** the job that checks invariants and left the other three green. A gate that
reds on everything satisfies "it went red once" and measures nothing.

### `hubot_live_stack/` — the harness, and why it ships here

The behavioural suite drives this filter through a live `LayeredCostmap` inside a gtest process. That
is not a robot, and it is not even a running navigation stack. `hubot_live_stack/` is the step up: it
starts five OS processes — a parameter target node, a scenario support node, a decision observer,
nav2's own `controller_server`, and a behaviour-tree driver loading nav2's real
`libnav2_clear_costmap_service_bt_node.so` — so that the costmap's own update thread and the filter's
independence from it are **exercised rather than argued for**.

⚑ **It is here because a safety document cited it and no reader could obtain it.**
`doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md` discharged its largest liveness claim by pointing at this
harness while the harness existed only on one machine, in no repository at all. Every reader followed
that citation to nothing.

⚑ **And when it was finally run against a released nav2, it failed** — the configuration named a
controller its manifest never declared, which a source-built image had been supplying by accident. One
declaration fixed it. **Its runner also used to exit `0` unconditionally**, which is how a total
lifecycle failure had been reported as three successful runs. Both are corrected, and the CI job above
is what keeps them corrected.

**What it still is not:** one host, simulated transforms, a synthetic mask, no sensor layers, and no
goal sent — so the costmap thread is real but the load on it is not. Not a robot. Not hardware.

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
…    up to 100
```

Paint the doorway area `1`, paint the loading bay `2`, leave everything else `0`. That
is the whole map side of it. The mask is published by nav2's standard
`costmap_filter_info_server` and `map_server` pair, exactly as it is for a keepout
filter; nothing new is involved.

> ### ⚑ Two things about that, measured 2026-09-06 through a real `map_server`
>
> **1. The ceiling is 100, and this page said 255 until it was measured.** In
> `mode: raw` nav2's map loader clamps any value outside `[0, 100]` to `-1`
> (`nav2_map_server/src/map_io.cpp:316-322`), and hubot treats a negative cell as
> UNKNOWN and does not change state. So `101` and above are not "zone 101" — they
> are "no reading", silently. The old sentence is kept here rather than deleted
> because a reader who painted `200` needs to know what they were told.
>
> **2. `mode: raw` in the mask's `.yaml` is not optional.** `trinary` and `scale`
> put every pixel through thresholds, which collapses every zone id to `0`, `100`
> or `-1`. The stack still comes up; every zone just reads as the same zone. A
> keepout mask is loaded `trinary` and a speed mask `scale`, so copying either
> one's yaml is the wrong move here.
>
> A worked example, with both of these in its own header, is installed at
> `maps/zone_mask.pgm` — see **Run it** below.

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

**`node:` may be absolute or relative, and both now behave the way you would expect.**
An absolute name (`/controller_server`, as every example above writes it) is used exactly
as given. A relative name (`controller_server`) is resolved **against the parent
namespace** — the identical rule this filter already applied to its own four topic names.

> ### ⚑ Two warnings about this example, both measured 2026-09-06
>
> **1. `nominal_defaults` cannot be written as nested YAML, and we do not yet have a
> form that is proven to load.** The filter reads `<filter>.nominal_defaults` as a
> **string array** (`src/zone_parameter_filter.cpp:337`) and also reads
> `<filter>.nominal_defaults.<name>.node` as children of that same key (`:339`). In a
> ROS 2 parameter file one key cannot be both a sequence and a mapping. The dotted
> form printed above is written the only way the two can coexist in one document, and
> **whether the parameter loader accepts it is untested.** Until it is, set those
> entries from a launch file or the command line, where the dotted names are exactly
> what the code reads. `states` does **not** have this problem — the state names are
> different keys from `states` itself.
>
> **2. The file form IS now exercised — that was not true until 2026-09-06.**
> This warning read, in full: *"No YAML file has ever configured this filter. The
> package contains zero `.yaml` files and zero tests that load parameters from
> one; every test sets the parameters programmatically. So the configuration path
> you would use is the one path this package has never exercised."* It is kept
> here because a reader of the previous page was told to distrust the file form
> and is owed the reason it changed.
>
> What changed: `params/zone_filter_demo.yaml` is a real parameter file, loaded
> by a real `Costmap2DROS` from a launch file, and the run reaches `enforced:
> yes` with the set arriving in the target's own process. See **Run it**.
>
> **What is still NOT verified from a file is `nominal_defaults` specifically** —
> the sequence-and-mapping collision in warning 1 above is unchanged and no yaml
> spelling of it has been made to load. The shipped launch file sets it as node
> parameters instead, which is the form the code reads and the only form measured
> to work. Everything else on this page's example loads from yaml.

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
`src/zone_parameter_filter.cpp:118` (`initializeFilter`), `:314` (`filterInfoCallback`) and
`:397` (`loadStateConfig`). None is on the `process()` / `updateCosts()` path that this
package is about, and the upstream filter carries the same three — but nothing above them
handles an exception either, so "nothing takes the node down" was more than we had shown.

That choice is deliberate and it has a cost, so it is stated plainly: a filter that
carries on has not fixed anything — it has handed you a decision you would not
otherwise have had. **Making that decision possible is the whole design.**

## The surface you read to decide — `zone_decision`

A `diagnostic_msgs/DiagnosticArray`, published on every zone transition, on every
enforcement failure, and every `liveness_period` in between.

⚑ **What we know about how it reaches you, and what we do not.** The **message type** is the
ROS-native diagnostic type, so no new interface package is involved and any tool that reads
`diagnostic_msgs/DiagnosticArray` can read these rows. **Two things beyond that are yours, and
we have measured neither:**

- **The topic is not `/diagnostics`.** It is `zone_decision`, joined to your costmap's
  namespace (`src/zone_parameter_filter.cpp:234-235`; the name is the `decision_topic`
  parameter). `diagnostic_aggregator` and `rqt_robot_monitor` subscribe to `/diagnostics` by
  convention, so **they will not find this topic without a remap.** A generic viewer pointed
  at the topic directly will.
- **The ranking is yours.** Nothing in these rows marks `enforced` as the field that decides
  whether a zone is holding. In a viewer it arrives among every other diagnostic on the robot,
  with no more prominence than a battery percentage.

**We have not measured how any third-party tool renders this, and this page will not tell you
that it does.** Every subscriber that exists today is one of this package's own tests or its
harness.

| field | carries |
|---|---|
| `level` | `OK` in force · `WARN` requested but not yet confirmed · `ERROR` when the zone is **not** being enforced · `STALE` when the filter is **not watching** |
| `message` | a sentence to act on, **naming the target** — *"Zone 2 is NOT being enforced on target '/controller_server'. Decide as if the zone's limits are not applied."* With more than one: *"...on 3 targets, including '/collision_monitor'."* |
| `values` | `zone_state`, `mask_state`, `enforced`, `configured`, `unconfirmed_targets`, `degraded_targets`, `pending_parameter_sets`, `targets`, the triggering `event`, and the liveness fields `watching`, `costmap_age_s`, `report_seq`, `report_period_s`, `valid_for_s` |

**`report_period_s`** is how often the next message is due — the heartbeat period actually in
effect, not the configured default. **`valid_for_s`** is how long *this* message should be
treated as current: `2.5 x report_period_s`, **clamped to `costmap_silence_timeout`** while
that detector is on — a message must never declare itself current past the age at which this
filter would call its own reading stale (at the shipped defaults that clamps 2.5 s to **2.0 s**,
and the filter logs one warning at startup naming both numbers). It is wide enough to survive one dropped or late
heartbeat without expiring a healthy publisher, and narrow enough that a dead one is stale
inside three periods. **If `now - header.stamp > valid_for_s`, stop believing the message** —
including its `enforced` field.

⚑ **That last check is yours, and the reason is narrower than "we cannot do it."** We do
publish expiry verdicts against our own declared bound — `watching`, `costmap_age_s` and level
`STALE` are exactly that, rendered on every message. What we cannot render is the verdict on
*this* message, because it only becomes true after the moment we would have to publish it: if
we are alive at that moment we have already sent you a newer message and the question is moot,
and if we are not alive we cannot send anything at all. **The one case where the check matters
is the case where we are gone** — and only something that outlives a message can judge that the
message expired. `report_seq` is there so a frozen clock cannot fake liveness; the clock is not
the obstacle, survivorship is. See the bound at the end of the liveness section.

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

⚑ **`unknown` was added 2026-09-06 and it is a breaking change to this vocabulary.** **The rule
that goes with it is a whitelist, and the old blacklist below has been corrected for the same
reason.**

⚑ **The justification this paragraph carried until 2026-09-07 has EXPIRED. It is corrected here
rather than deleted.** It read: *"It is made now, deliberately, because this package has no remotes
and no consumer can be holding the old set — the cheapest moment it will ever be."* That was true
when it was written on 2026-09-06. **It is false now:** this repository is public and has published
`0.1.0` and `0.1.1`. The change itself stands and is not being re-litigated — but **the reason it
was cheap has lapsed**, and any future change to this vocabulary must be treated as breaking a
consumer who may exist, not as free. A justification that expires unread is how a page goes on
arguing for something the world has already moved past.

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

**⚑ It builds against released nav2 `1.5.1` — measured, not assumed — and against branch HEAD.**

The blocker that stood here this morning was one symbol, `nav2_costmap_2d::ZONE_PARAMETER_FILTER`,
absent from every release tag. Patching past it and rebuilding the entire nav2 chain from tag
`1.5.1` showed **it was the only blocker**, at eight sites; the three candidates for a second one
were refuted by that build. It is replaced by `hubot::kZoneParameterFilterType`, `static_assert`-
guarded on every nav2. The record of what it was — the compiler transcript, the check, what
changed — is in the section near the top of this page and in the changelog, not repeated here.

**What is and is not verified on that release.** The library and every test binary build; 23 of
26 tests pass; the plugin resolves and runs through a live `LayeredCostmap`. The three that fail
are one pre-existing case identical on branch, and two whose *upstream-comparison* control loads
a plugin that exists only on branch. `1.5.0` is not built and not claimed. `rmw_fastrtps_cpp`
only.

⚑ **A version number will not tell you which nav2 you have.** Tag `1.5.1` (`a6354f3f`) and
`lyrical` HEAD (`6f23b11c`) both declare `<version>1.5.1</version>`. It no longer decides whether
this package builds — it decides whether the build cross-checks our discriminator against
upstream's. Check for the constant, not the number:

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

`Layer::setCurrent(bool)` is **public** (`layer.hpp:147`), so the capability exists in the base
class and is erased by *statement order* in the derived one, not by an architecture. What you
must do is the same either way.

**So: subscribe to `zone_decision` and ⚑ PROCEED ONLY ON `enforced: yes`.** If you do not,
hubot has told you and nothing has listened.

**It has never run on a robot.** Verified 2026-09-06 from a clean workspace against
**upstream `lyrical` branch HEAD `6f23b11c`, with no local nav2 patches on the path**:
`colcon build` green, `ldd -r` reports zero undefined symbols, `pluginlib` resolves
`hubot::ZoneParameterFilter` out of the ament index, and **28 of 28 tests pass** (21 before
2026-09-06; `colcon test-result --all`) — including a
live `LayeredCostmap::updateMap()` and a **negative control that requires upstream's own
filter, loaded from that same build, to throw out of `updateMap()` on the same input**. It
does (`"ZoneParameterFilter: set_parameters failed: parameter 'readonly_speed' cannot be set
because it is read-only"`), so the harness can see the failure it rules out.

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
in that case.

⛑ **THIS PARAGRAPH DESCRIBED THIS PACKAGE UNTIL THE SILENCE DETECTOR EXISTED, AND IT SAID
THE FOLLOWING IN THE PRESENT TENSE:** *"An `OK` from ninety seconds ago renders identically to
an `OK` from now. You would read the zone is enforced. The truth would be nobody is checking."*

**That is the failure this package was written to abolish, and it was inside the package.**
**It is no longer what this component does, and the sentence above was still telling you it
was.** Measured on this tree: a costmap silent past its budget sets level **`STALE`**, not
`OK` (`src/zone_parameter_filter.cpp:1189`), and the message becomes *"THIS FILTER IS NOT
WATCHING."* A ninety-second-old reading does **not** render identically to a fresh one.

⛑ **And the residual is not the one you would guess.** Switching the detector off with
`costmap_silence_timeout: 0` does **not** restore it: since 2026-09-06 the budget falls back to
`liveness_period x 2.5` (`src/zone_parameter_filter.cpp:1069-1072`), so `STALE` still fires.
**The window that genuinely survives is the one no detector inside this process can close — if
the whole node dies, nothing publishes and the last message stands.** That is what `valid_for_s`
and the offered `DEADLINE` are for, and it is stated in full above.

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
| `costmap_silence_timeout` | `2.0` s | switch off the **explicit** stopped-costmap report; a fallback budget of `liveness_period x 2.5` still applies |

⚑ **`costmap_silence_timeout: 0` changed meaning on 2026-09-06, and the old meaning was a
defect.** It used to mean *never report a stopped costmap*, and it delivered that by making
the refuting flag unreachable — so a filter whose costmap had died an hour ago went on
publishing `watching: yes`, `enforced: yes`, level `OK`, forever, from one line of YAML.
**Suppressing a warning and asserting safety are different acts.** Disabling the detector
now suppresses the explicit report and falls back to the same `x 2.5` budget that defines
`valid_for_s`; it no longer licenses an affirmative claim that the costmap is running. If
you are actively driving the costmap you still read `yes` — only the unfounded claim went
away.

⚑ **THE PART THIS DOES NOT CLOSE, AND CANNOT.** **If the node itself dies, the timer dies
with it**, no message arrives, and the last one stands — exactly as before. **Nothing
running inside a process can announce that process's own death.** What you get instead of a
bare stamp is a **declared expiry**: every message carries `report_period_s` and
`valid_for_s`, so three lines of consumer code can decide the report has expired, and
`report_seq` advances on every publish so a frozen `/clock` cannot fake liveness. **That is
strictly better than "check it". It is still something you must check.**

**The complete answer lives in your process, and a `DEADLINE` QoS on your subscription now
works against this publisher** — it raises the alarm in *your* node when ours goes quiet,
including the case this package cannot otherwise reach, the node dying with its last
message left standing.

**We offer a deadline equal to `valid_for_s` — `liveness_period x 2.5`, clamped to
`costmap_silence_timeout`.** One promise, stated on two channels, computed in one place, so they
cannot drift apart. At the shipped defaults that is **2.0 s** (the clamp), not 2.5 s.

⚑ **The trap, and it is the opposite of what you would guess: `DEADLINE` is a
Request/Offered policy, and the offered period must be *less than or equal to* the
requested one. So you must request a deadline NO SHORTER than ours.** Request `2.0 s` or
more at default settings and you match. Request `1.0 s` — a stricter, more cautious value —
and the subscription **silently does not match and you receive nothing.** Set an
`incompatible_qos_callback` on your subscription and you will be told; without one, an
over-strict request is indistinguishable from a dead publisher.

⚑ **BOUND: measured on `rmw_fastrtps_cpp` only** (`test/sotif_gate_inertness_test.cpp`
SC-5, two arms, the default-QoS control required to receive or the deadline arm proves
nothing). QoS matching is RMW-dependent and this result does not generalise to your stack.
**If `liveness_period <= 0` the heartbeat is off, no rate can honestly be promised, and no
deadline is offered** — do not request one in that configuration.

## The three-letter tags in the comments

Source and test comments here are signed with initials — `CPP`, `FSE`, `AAA`, `FBR`,
`DIA`. They are authoring roles on this package, nothing more: C++ and engine work,
functional safety, safety-standards review, build reproduction, document integrity.
A tag records who found or wrote a thing so a later reader can tell one hand from
another in a file with a long correction history.

**They are not references to anything outside this repository, and nothing here asks
you to look them up.** They are stated because an unexplained initial invites a search
that cannot succeed — and a citation a reader cannot resolve is worse than none, since
it looks resolved. The same rule is why Sakichi's principles are quoted in full in the
comments that rest on them rather than cited by a number you would have no way to check.

## Lineage and licence

hubot's filter derives from `nav2_costmap_2d::ZoneParameterFilter` and is written by
the same author, with the same Apache-2.0 licence and the same copyright line:
**Copyright (c) 2026 Komada (aki1770-del)**. The upstream filter's design intent —
that a failed parameter set must never be silently swallowed, because that leaves a
robot on the value a safety zone tried to change — is correct, is kept here
verbatim in the source, and is the reason the reporting surface exists at all.

Findings raised on the upstream filter by nav2's maintainer are **not fixed upstream** by
anything in this package — if you run the upstream filter, those remain its business.

Three of them describe a design this package shares. They were checked one by
one against this source, and the result is a table in `doc/SPEC_COVERAGE.md` §4 giving a
verdict and its evidence for each. In short: the `reset()`/`deactivate()` conflation **is here**
and cannot be fixed by overriding, because `CostmapFilter::reset()` is `final`; the
event-topic ordering gap **was here and is fixed**; the parameter-client rebuild **is here**,
with its discovery-race consequence **UNVERIFIED, not cleared**; and the re-apply arming window
**cannot occur here** because there is no re-apply mechanism at all. Each of those verdicts has a
grep behind it, and the ones that occur have a test that fails without the fix.
