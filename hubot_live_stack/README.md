# `hubot_live_stack` — the harness where hubot's independence claim stops being simulated

**Seat:** BDE (build-debug-engineer). **Date:** 2026-09-06.
**Class:** SUBSTRATE. This package **asserts nothing** about `enforced` / `watching`
semantics. Those assertions are FSE's and are added on top of what runs here.

---

## The gap this closes

Every `hubot` test drives `updateCosts()` **by hand from the test thread** against a
bare `LayeredCostmap`. Measured 2026-09-06 across all eight files in `hubot/test/`:
`Costmap2DROS` is **never instantiated** — not once. It occurs 9 times, and every one
is a comment or an assertion-message string literal citing what production does;
`grep -n Costmap2DROS hubot/test/*.cpp` returns 8 comment lines plus one string at
`pluginlib_live_costmap_test.cpp:445`. The class the package's whole warrant rests on
is discussed everywhere in that suite and constructed nowhere in it.

⚑ *This paragraph first said "appears zero times", which is false and a reader running
that grep would have caught it. Corrected in place rather than deleted (OPS-066(B)):
the honest figure is 9 mentions and 0 instantiations, and the second number is the
one that carries the argument.*

The warrant for hubot's liveness countermeasure is that
`Costmap2DROS::map_update_thread_` (`costmap_2d_ros.cpp:314`) and the node's executor
thread (`controller_server.cpp:72`, `nav2::NodeThread`) **fail independently**. Both
lines were read and verified verbatim at those exact line numbers. That claim had
been read and never *exercised*: the fixture simulated the independence it exists to
prove.

Here, nothing is simulated.

## What actually runs — five OS processes

| # | process | what it makes real |
|---|---------|--------------------|
| 1 | `param_target_node` | the filter's parameter target, **its own process**, answering on nobody's schedule |
| 2 | `scenario_support_node` | latched `CostmapFilterInfo` + mask + continuous TF |
| 3 | `decision_observer` | reads `/zone_decision` **off the wire**, writes a JSONL transcript |
| 4 | **`controller_server`** | **the real nav2 binary** — `nav2::NodeThread` at `:72`, `map_update_thread_` at `:314` |
| 5 | `bt_clear_costmap_driver` | a real `BT::BehaviorTreeFactory` loading nav2's own `libnav2_clear_costmap_service_bt_node.so` |

The recovery path is genuine end to end:

```
BT tick -> nav2_behavior_tree::ClearEntireCostmapService
        -> nav2_msgs/srv/ClearEntireCostmap over the middleware
        -> nav2_costmap_2d::ClearCostmapService   (clear_costmap_service.cpp:63)
        -> Costmap2DROS::resetLayers()            (costmap_2d_ros.cpp:719)
        -> Layer::reset() on every filter         (costmap_2d_ros.cpp:735)
        -> CostmapFilter::reset() = resetFilter(); initializeFilter(...)
```

and it arrives on the **costmap node's executor thread** while `map_update_thread_`
may be inside `updateCosts()`. A direct `resetFilter()` call reproduces neither the
service hop nor that contention.

## Where this package lives, and the one thing that will surprise you

This package is a directory **inside** the `hubot` package's own repository. That
repository's ROOT carries a `package.xml`, and **colcon's recursive crawl stops at
the first `package.xml` on a path** — so from an ordinary workspace this package is
invisible. Measured 2026-09-07, ubuntu 26.04 + colcon-core 0.21.1, with
`src/hubot` a clone of this repository:

```
$ colcon list --names-only
hubot                                  # and nothing else

$ colcon build --packages-select hubot_live_stack
WARNING:colcon.colcon_core.package_selection:
    ignoring unknown package 'hubot_live_stack' in --packages-select
Summary: 0 packages finished [0.07s]
rc=0
```

⚑ **Read that exit code again.** Asking colcon to build this package builds nothing
and returns zero. `rosdep` follows the same crawl rule: `rosdep install --from-paths
src/hubot` resolves 17 keys and does not see this package's three extra ones, and
`rosdep` pointed at *this directory alone* fails outright —
`ERROR: hubot_live_stack: Cannot locate rosdep definition for [hubot]`, because
`hubot` is an `<exec_depend>` here and is not a published rosdep key.

**Both tools must be given both paths explicitly.** That is what
`scripts/build_gate.sh` does, and it asserts the resulting install space rather
than trusting the exit code.

## Reproducible invocation — released nav2, no image you cannot obtain

```bash
mkdir -p ws/src && cd ws
git clone https://github.com/aki1770-del/hubot src/hubot

rosdep install --from-paths src/hubot src/hubot/hubot_live_stack \
  --ignore-src --rosdistro lyrical -y

bash src/hubot/hubot_live_stack/scripts/build_gate.sh "$PWD"

. install/setup.bash
bash install/hubot_live_stack/share/hubot_live_stack/scripts/run_live_stack.sh \
  --outdir "$PWD/live_stack_run" --hold-secs 10
```

Artifacts land in the `--outdir`: `transcript.jsonl` (the observable),
`controller_server.log`, `param_target.log`, `bt_recovery.log`, `graph.log`.

The same three conditions run in CI on every push — `gate.yml`'s `live-stack` job,
on `ubuntu:26.04` with released nav2 from `packages.ros.org`. If you want a run you
did not have to trust anyone for, read that job's log.

### Conditions — one base, overlays applied after it

The base and its controls share **one** configuration so they cannot drift apart.

| overlay | condition |
|---------|-----------|
| *(none)* | target parameter is `read_only`; rclcpp refuses before any user callback |
| `overlay_target_accepts.yaml` | **positive control** — writable parameter, the set reaches the target's own callback and is logged by it |
| `overlay_target_relative_name.yaml` | the namespace miss (below) |

```bash
run_live_stack.sh --overlay overlay_target_accepts.yaml --outdir /tmp/run_accept
```

Process-level conditions are flags, not code changes:
`--target-args "--freeze-after-ms 4000"` (target hangs, service stays in the graph),
`--target-args "--exit-after-ms 4000"` (target leaves the graph),
`--target-args "--reply-delay-ms 4000"` (answers after the deadline),
`--support-args "-p stop_tf_after_ms:=6000"` (TF goes stale, so the **real** update
loop stops driving the filter without the node dying — stopped-vs-quiet, produced by
nav2 rather than by not calling something).

---

## Three things the harness found by being run

These are build-and-topology facts, measured, each with a control. What they *mean*
for safety is FSE's to rule on.

### 1. ⚑ A relative `node:` name targets a node that does not exist in production

`rclcpp::AsyncParametersClient` (`zone_parameter_filter.cpp:387`) resolves the target
node name **relative to the owning node**. In production that node is always
namespaced — nav2 makes it `/local_costmap/local_costmap` under `controller_server`.
So a bare `zpf_target_node` builds a client for
`/local_costmap/zpf_target_node/set_parameters`, against a target living at
`/zpf_target_node`.

Two-sided control, measured:

| | relative `zpf_target_node` | absolute `/zpf_target_node` |
|---|---|---|
| target process log | **nothing arrives** | `set_parameters: writable_speed -> 0.5 (successful=true)` |
| filter transcript | `enforced: NO` only after the 3.0 s deadline | `pending` → `yes` |
| external `ros2 param set /zpf_target_node` | **succeeds** — the target is reachable | — |

**Every in-process hubot fixture hosts its target on a node at root namespace, where
a relative name resolves correctly by accident.** The class cannot appear there, and
did not.

### 2. ⚑ With no bounds-declaring plugin the filter is driven ZERO times, and everything looks green

`CostmapFilter::updateBounds()` ignores all four bounds arguments. With no *plugin*
declaring a window, `LayeredCostmap::updateMap()` computes `Updating area x: [9, 1]`
and the `if (xn >= x0 && yn >= y0)` guard around every `(*filter)->updateCosts(...)`
is never satisfied. Measured: lifecycle configure and activate both `rc=0`,
`map_update_thread_` logging `Map update time` at **5.000 Hz**, footprint publishing
at 5 Hz — and the filter driven **not once**. `FullWindowBoundsLayer` exists for this.

### 3. ⚑ `Layer::enabled_` is uninitialized, and reading it silently disabled a layer

`Layer::initialize()` (`layer.cpp`) never touches `enabled_`, declared
`bool enabled_;` at `layer.hpp:200`. Every concrete nav2 layer declares it itself
(`inflation_layer.cpp:94`). Our first layer did not, read an indeterminate `false`,
and produced defect 2 above. **hubot's own liveness signal is what caught it**, from
inside the running stack: *"NOT WATCHING. This filter has not been driven even once."*

*(No nav2 source was modified. Per BDE's charter this seat does not patch upstream.)*

---

## Honest bounds — UNVERIFIED, not cleared

- **Not a robot, and not hardware.** One container, one host, `--network=none`,
  simulated TF and a synthetic mask. Timing on real hardware is not asserted.
- **Single-host verified only.** Cross-machine and multi-host DDS are untested.
- **No sensor layers.** `plugins:` carries only the bounds layer, so contention
  between the filter and a real obstacle layer is **not** exercised.
- **The controller's control loop is idle.** No goal is sent, so `FollowPath` never
  computes velocity commands. The costmap thread is real; the *load* is not.
- **`--network=none` by design.** Nothing here reaches any network, ever. The CI
  job runs in a GitHub container that does have a network; discovery is confined by
  `ROS_LOCALHOST_ONLY` and a per-run `ROS_DOMAIN_ID` instead.

## ⚑ The substrate is part of the result, and for a day this file did not say so

The first run of this harness, 2026-09-06, happened on a **local, unpublished
container image** in which every nav2 package had been built **from source** —
`nav2_costmap_2d` 1.5.0 in one overlay and 1.5.1 in another, with **no released
nav2 debs installed at all**. Its three conditions returned `configure rc=0`,
`activate rc=0`, `bt_recovery rc=0`, and that result is true of that image.

⚑ **It was not true of released nav2, and nothing here would have told you.**
Re-run 2026-09-07 on ubuntu 26.04 resolute with released nav2 1.5.1 from
`packages.ros.org`, this harness got as far as bringing the costmap and the filter
up — `Initialized costmap filter "zone_parameter_filter"` — and then died:

```
[FATAL] [controller_server]: Failed to create controller. Exception: According to the
loaded plugin descriptions the class
nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController with base class
type nav2_core::Controller does not exist. Declared types are
```

`config/live_stack.yaml` named that controller; `package.xml` did not declare the
package that provides it. On an image where all of nav2 was built from source it
resolved by accident. One `<exec_depend>` line closed it, and the same three
conditions then returned `rc=0` throughout on the released substrate.

**The general form: a harness whose result you cannot reproduce on a substrate you
can obtain has told you about the image, not about the code.** That is why the
invocation above names released nav2 and why CI runs it.
