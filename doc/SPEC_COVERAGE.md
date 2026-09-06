# Hubot — specification coverage: FIT and GAP

**Date**: 2026-09-05. **Method**: every row below was read from the artifact on disk
this turn, not from what the author intended. Nothing here is a plan.

---

## ⚑ GAP-0 — SUPERSEDED 2026-09-06. IT BUILDS, AND EVERY ROW BELOW IS NOW FALSE.

**This section was true when written (2026-09-05) and is false now.** It is corrected here
rather than deleted, because a reader needs to know the package once could not be compiled
at all and what that concealed.

**Re-measured 2026-09-06 against upstream nav2 `lyrical` HEAD `6f23b11c`, clean workspace:**
`colcon build` green, `colcon test` **35 tests, 0 errors, 0 failures, 0 skipped**.

| symbol | GAP-0 said | measured 2026-09-06 |
|---|---|---|
| `enterState()` | declared, not defined | **0 occurrences in either file** |
| `reapplyAfterDrainIfDue()` | declared, not defined | **0 occurrences in either file** |
| `reapply_after_drain_` | dead member | **0 occurrences in either file** |
| `kMaxPendingSets` | dead constant | **live: 1 decl + 3 uses** |
| `pending_sets_` | container nothing fills | **live: 12 references, filled by `issueAsyncSetParameters()`** |

⚑ **The row GAP-0 called the worst of them is closed.** `publishDecision()` reports
`pending_parameter_sets` from `pending_sets_.size()`, and `pending_sets_` is now the container
the code actually fills. `pending_futures_` no longer exists anywhere.

**GAP-0's own diagnosis was right and is worth keeping:** *"None of this surfaced because
nothing has been compiled."* The version conflict it named — a header from one version against
a source from another — was resolved by choosing one, as it said it must be.

**The build bound that DOES stand, and it is a different one:** this package compiles against
nav2 branch HEAD (`main` or `lyrical`) and — ⚑ **REVERSED 2026-09-06, later the same day** —
**now also against release tag `1.5.1`.** The bound had read *"NOT against any released nav2,
because `src/zone_parameter_filter.cpp:289` needs `nav2_costmap_2d::ZONE_PARAMETER_FILTER`."*
Measured: that symbol was the only blocker, at eight sites, and it was a copying mistake — a
wire discriminator we imported instead of declaring. It is now `hubot::kZoneParameterFilterType`
(`include/hubot/zone_parameter_filter.hpp:64`), `static_assert`-checked on every nav2. Built
and installed against tag `1.5.1` with the full nav2 chain from that tag; 23 of 26 tests pass
there, and the three that do not are SC-3 (identical on branch) and the two pluginlib arms whose
*upstream-comparison* control needs a plugin that exists only on branch. `1.5.0` is not built
and not claimed. The old text is kept because it is the reason the new text exists.

---

## 1. Repository and packaging

| specification | state | FIT / GAP |
|---|---|---|
| out-of-tree ROS 2 package installable without an upstream merge | ⚑ **CORRECTED 2026-09-06.** The `ament_cmake` layout and `pluginlib` export are present and correct — **but they were the wrong evidence for this row.** A plugin export says nothing about whether the translation unit compiles, and it does not, against any release: `src/zone_parameter_filter.cpp:198` needs `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, absent from every release tag. The row asked whether an integrator can install it and was answered with whether we packaged it. ⚑ **REVERSED 2026-09-06, later the same day:** the symbol is replaced by `hubot::kZoneParameterFilterType`, the translation unit compiles against release tag `1.5.1`, and the plugin resolves and runs through a live `LayeredCostmap` on that release (`pluginlib_live_costmap_test` B and B2). This row asks about an upstream *merge*; it does not ask whether the package is *obtainable*, which is a separate bound (zero remotes) and the reach seat's to rule on. | **FIT** — was ⚑ **GAP** — was **FIT** |
| licence clean | Apache-2.0; the upstream file's own header reads `Copyright (c) 2026 Komada (aki1770-del)` — we are the holder | **FIT** |
| reachable by an integrator | nothing is published; repo is local only | **GAP** — a publish is Chair-only |
| C++ estate | this is our first C++ repository; measured, we owned zero | **FIT**, newly |

## 2. Feature 1 — degrade instead of abort

| specification | state | FIT / GAP |
|---|---|---|
| a failed parameter set must not kill the navigation node | the throw is replaced by ERROR log + latched flag | **FIT** in code, **UNVERIFIED** — never compiled or run |
| never silently swallow the failure | every failure logs at ERROR with its reason | **FIT** in code |
| the condition is inspectable by an integrator | `enforcementDegraded()` is public | **FIT** |
| proof the fix works | 2 construction-level assertions only | ⚑ **GAP** — the three cases that would fail on the real defect need a live executor and a rejecting target node. **Owed, not written.** |
| the upstream defect itself | static call-chain read at both ends | **GAP** — never reproduced on a running robot |

## 3. Feature 2 — `zone_decision`, the human-decision surface

| specification | state | FIT / GAP |
|---|---|---|
| the human needs the basis, not the state number | `diagnostic_msgs/DiagnosticArray` beside the robot's `UInt8` | **FIT** in design |
| say plainly when the zone is NOT in force | `level = ERROR` + an actionable sentence | **FIT** in design |
| emitted when a decision is due | on transition and on every enforcement failure | **FIT** in design |
| the robot's surface is not taken away | `UInt8` topic untouched | **FIT** |
| no new interface package | standard `diagnostic_msgs` | **FIT** — Precept 3, no ornament |
| the values must be true | `pending_parameter_sets` reads a container nothing fills | ⚑ **GAP — it would publish a false zero** (see GAP-0) |
| the topic is configurable | `decision_topic_` is **hardcoded**; every other topic is a declared parameter | ⚑ **GAP** |
| alive when it is most needed | `decision_pub_` is torn down in `resetFilter()` — the same shape the maintainer flagged upstream for the state-event publisher | ⚑ **GAP** — the surface goes silent at the moment a reset drops the zone |

## 4. The maintainer's six findings, one verdict each, measured against THIS package

⚑ **CORRECTED 2026-09-06. This section previously said, in full:** *"his six findings are about
the changes PR #6372 proposes, not about the released code. Hubot is built from the released
file, so **none of the six is present in Hubot**, and none of the six is fixed by Hubot either.
They are simply not this package's subject."*

**The last clause was reasoning, not measurement, and it was wrong.** Three of the six ARE present
here — one of them character-for-character — and the sentence that hid them is the exact sentence
under which a defect survives a sweep. The old text is kept above because it is the reason this
table now exists.

**Method.** Each finding was checked against `src/zone_parameter_filter.cpp` and
`include/hubot/zone_parameter_filter.hpp` on disk this turn, by grep with the counts shown. A
finding is only marked **cannot occur** when the *mechanism it names is absent*, and the reason
is stated so a later reader can re-test it rather than trust it.

| # | his finding | verdict here | evidence |
|---|---|---|---|
| 1 | `resetFilter()` calls `applyState(0)` when in a non-zero state, so a routine clear un-enforces the zone | **mechanism cannot occur; the CONSEQUENCE did, by another route** | `applyState` inside `resetFilter()`: **0 occurrences**. This filter does not restore on reset — so the zone's values *stay* on the target while the filter reports state 0. The lie inverts direction and the human-decision surface is wrong either way. Caught by `MF3`. |
| 1b | `reset()` and `deactivate()` both land in `resetFilter()` and the plugin cannot tell them apart; `Layer::reset()` is virtual, so override it | **OCCURS HERE — and his remedy is impossible** | `void reset()` override in this header: **0**. But `CostmapFilter::reset()` is declared `final` (`costmap_filter.hpp:127`), so a subclass cannot override it; g++ rejects it. `CostmapFilter::deactivate()` is `resetFilter()`; `reset()` is `resetFilter(); initializeFilter(...); setCurrent(false);`. **No behavioural defect follows in this package** — the two paths differ only in what is rebuilt afterwards — so there is no failing test for it, and that is stated rather than papered over. |
| 2 | `state_event_pub_` is destroyed *before* the state change, so the change is silent on the event topic | **OCCURRED HERE; FIXED 2026-09-06** | Same ordering was present: both publishers were destroyed at the top of `resetFilter()` and every state mutation followed. `resetFilter()` now clears state, publishes one farewell describing the cleared state, and destroys the publishers last. Caught by `MF4`. |
| 3 | `param_clients_.clear()` + rebuild means a brand-new `AsyncParametersClient` per target on every clear | **OCCURS HERE, code shape identical. Trigger UNVERIFIED** | `param_clients_.clear()` at `:1430` inside `resetFilter()`; `param_clients_.emplace(...)` at `:491` inside `loadStateConfig()`, which `initializeFilter()` re-runs. So every `ClearEntireCostmap` destroys and rebuilds every client. **The discovery-race consequence is his engineering claim and this package has NOT reproduced it** — UNVERIFIED, never *cleared*. |
| 4 | `sets_in_flight_before_restore` is sampled *before* the restore, so the restore's own sets never arm the re-apply | **cannot occur — no re-apply mechanism exists** | `sets_in_flight_before_restore`: **0 occurrences**. There is no drain-re-apply here at all; re-application is driven by the mask returning and `process()` running, not by a latch. ⚑ **The generalised form DOES occur and is tested:** `resetFilter()` discards `pending_sets_` and `unconfirmed_targets_` while those requests are still on the wire, so the filter forgets sets it issued. Caught by `MF5`. |
| 5 | `reapply_after_drain_ = …` is an assignment, not an OR, so a second clear discards the re-apply owed by the first | **cannot occur — the flag does not exist** | `reapply_after_drain_`: **0 occurrences**. Nothing here accumulates work across two clears; `resetFilter()` is idempotent. |
| 6 | `reapplyAfterDrainIfDue()`'s `state_still_configured` guard is effectively dead | **cannot occur — the function does not exist** | `reapplyAfterDrainIfDue`: **0**; `state_still_configured`: **0**. ⚑ **The generalised form — a guard the caller can walk around — DID occur:** the `!state_initialized_` guard in `publishDecisionOnStatusChange()` was bypassed by `livenessTick()` calling `publishDecision()` directly. Caught by `MF1`, and fixed in the *value* rather than at the caller. |

**Three of six occur here in some form; three cannot, each for a named and re-testable reason.**
The findings themselves are against the upstream filter's proposed changes and none of them is
*fixed upstream* by anything in this package.

**FIT**: this package's one behavioural change targets a defect that IS in the released upstream
filter — the abort. **GAP**: everything else on that thread remains upstream's.

⚑ **On the count.** He is variously recorded as raising 5, 6, 7, 8, 11 or 13 findings depending
on which round and which grouping is counted. **6** is the count for the third review round with
its root-cause limb listed separately (7 rows), and that is the set this table covers. **13** is
the count across all three rounds. Neither 7 nor 8 is written down as a headline anywhere; both
are reconstructions, and this file uses neither.

## 5. Instruments armed today, outside the package

| specification | state | FIT / GAP |
|---|---|---|
| measure defects found at Vision 15's worst rung | `downstream_catch()`, 8/8 two-sided, 2 mutants fail | **FIT** — first live number: 22 findings across 5 items, 4 answered inline |
| ratio of ours-caught vs theirs-caught | not measurable from the API | ⚑ **GAP** — numerator only, and the report says so |
| refuse self-proclaimed labels in maintainer-facing text | armed in the live gate, 8/8, quoted text exempt | **FIT** |
| a human's judgment arriving in the human's own voice | **no instrument, deliberately** — a gate for it is satisfiable by the pen | **GAP, and it is meant to stay one** |
| ruling banner reports only current rulings | fixed, 5/5, mutant fails | **FIT** |

## 6. The honest summary in one line

⛑ **CORRECTED 2026-09-06. This line said:** *"Design: covered. Code: written. Build: never
attempted, and the one measurement I did on the committed tree found that it would not have
succeeded."* **It was refuted twenty lines up, in this same file, by `:15`** — `colcon build`
green and `colcon test` 35/0/0/0 against `lyrical` `6f23b11c`, plus 23 of 26 against tag `1.5.1`.
A summary that contradicts the measurement printed above it is the shape this document exists
to catch, and it survived because nothing re-derived it.

**Design: covered. Code: written. Build: attempted and green** (see `:15`). Everything in
column FIT above that is not covered by a named test is still a claim about source rather than
about behaviour.

### The one place a test count is stated about THIS tree

⛑ **Every figure below is re-derived by `test/prose_matches_tree.py` (CHK-3) and the check
goes RED if this line and the tree disagree.** It is written here once, and nowhere else,
because four figures about this package's tests were published simultaneously and no two
agreed — `28 of 28`, `23 of 26`, `35 tests`, and a `46` that was in fact `colcon test-result
--all` counting a different quantity without saying so.

**DERIVED FROM THE TREE: 38 gtest cases in `test/*.cpp`, 9 CTest targets.**

**These are counts of what EXISTS, not results.** A pass/fail figure is only meaningful with
the tree it ran against, and the honest ones already carry it: *35/0/0/0 against `lyrical`
`6f23b11c`* (`:15`), *23 of 26 against tag `1.5.1`* (`:40`). Those are different quantities
measured on different stacks, they are allowed to differ, and this gate does not adjudicate
them — it requires only that **a bare number claiming to count this package's own tests is
this one.** ⚑ The 9th CTest target is `prose_matches_tree` itself, added 2026-09-06; it was 8
before. A gate that changes the number it checks must derive that number, never carry it.
