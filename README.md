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
keeps ticking. Nothing is swallowed and nothing takes the node down.

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

**⚑ hubot cannot stop your robot, and you must supply that yourself.**

nav2's channel for "this layer's output is untrustworthy" is `Layer::isCurrent()`,
which `ControllerServer::waitForCostmap()` gates on before terminating a goal. That
channel is closed to a derived filter three ways, measured 2026-09-05 on released
`lyrical`: `CostmapFilter::updateCosts()` calls `setCurrent(true)` unconditionally
after `process()` returns; `updateCosts()` is declared `final`, so the override does
not compile; and `Layer::isCurrent()` is not virtual. `enforcementDegraded()` is
public and nothing in nav2 calls it.

**So: subscribe to `zone_decision` and refuse to drive on `enforced: NO` or
`pending`.** If you do not, hubot has told you and nothing has listened.

**It has never run on a robot.** It builds green against ROS `lyrical` with
`nav2_costmap_2d` 1.5.1 and the plugin library `dlopen`s with zero undefined
symbols; the behaviour above is verified through `CostmapFilter::updateCosts()`,
the caller production uses. That is a gtest process, not a vehicle. Nothing here has
run on real hardware.

**A confirmed set means the target accepted the value, not that it still holds it.**
Anything else may set the same parameter afterwards and this filter will not notice.
Unverified by construction.

**It only learns what your targets said when the costmap ticks.** The result check
runs from `process()` and nowhere else, because a costmap filter owns no timer. If
costmap updates stop, `pending` never resolves, the deadline never fires, and the
last published value simply stands. The `DiagnosticArray` carries a header stamp —
check it.

**It is QM-class software. Nothing here is safety-certified.**

## Defects found in this package's own honesty, and fixed

Recorded because a package about trustworthy reporting has no standing to hide its
own lapses. All five were found and fixed 2026-09-05.

- `applyState()` threw on a mask value no state declared — the same abort this
  package removes from the parameter path, still open on the mask data path, where
  one mis-painted pixel reaches it.
- Removing that throw was the easier half. The undeclared value was still recorded
  as the current state, so the next transition's reset missed and the previous
  zone's limit rode into a zone that never asked for it. Measured: 0.2 m/s where
  1.0 was required.
- `resetFilter()` cleared the fault flag on the stated ground that the configuration
  it referred to was gone. The configuration was never cleared at all, while
  `CostmapFilter::reset()` re-runs the config load over it and `nominal_defaults_`
  is filled with `push_back`. So `ClearEntireCostmap` — which sits in seven of
  nav2's default behaviour trees — erased the fault while the fault stood, and
  doubled the nominal-defaults list every time it ran.
- `enforced: yes` was published on the same cycle the sets were issued, with zero
  confirmations. The code applied its own principle leaving a zone and violated it
  entering one.
- A target that never answered was reported as enforced, forever. `kMaxPendingSets`
  was declared to bound exactly that and referenced zero times — and could not have
  caught it anyway, since one silent set sits at a count of one indefinitely. A
  deadline does that job now.

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
