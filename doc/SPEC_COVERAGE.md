# Hubot — specification coverage: FIT and GAP

**Date**: 2026-09-05. **Method**: every row below was read from the artifact on disk
this turn, not from what the author intended. Nothing here is a plan.

---

## ⚑ GAP-0 — THE PACKAGE AS COMMITTED WILL NOT BUILD, AND THIS IS THE HEADLINE

I built the source from the **released `lyrical`** file and the header from the
**PR #6372 head**. They are two different versions of the class. Measured on the
committed tree:

| symbol | declared in header | defined in source | consequence |
|---|---|---|---|
| `enterState()` | yes | **NO** | link failure if reached |
| `reapplyAfterDrainIfDue()` | yes | **NO** | link failure if reached |
| `reapply_after_drain_` | yes | **0 references** | dead member |
| `kMaxPendingSets` | yes | **0 references** | dead constant |
| `pending_sets_` | yes | **1 reference — mine** | container nothing fills |

⚑ **And the last row is the worst of them.** `publishDecision()` reports
`pending_parameter_sets` from `pending_sets_.size()`. The released source uses
`pending_futures_`. **So the human-decision surface would publish a hard zero
forever, and it would look like an answer.** Sakichi Vision 14: *"a function that
returns a success-shaped value while the operation failed."* I built that defect
into the feature whose entire purpose is to stop the machine hiding what it could
not do.

**None of this surfaced because nothing has been compiled.** That is what "not
built" was concealing, and it is why the bound had to be stated rather than felt.

**FIT**: the package layout, manifests, plugin export and licence are correct and
complete. **GAP**: the class does not hold together. One version must be chosen —
released or PR-head — and the other discarded. **FBR/BDE own the build.**

---

## 1. Repository and packaging

| specification | state | FIT / GAP |
|---|---|---|
| out-of-tree ROS 2 package installable without an upstream merge | `ament_cmake` + `pluginlib` export present | **FIT** |
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

## 4. What the maintainer's six findings do and do not touch

⚑ **Measured correction to an assumption I nearly shipped**: his six findings are
about the changes **PR #6372 proposes**, not about the released code. His own
sentence — *"resetFilter() **now** does applyState(0)"* — is about the PR. Hubot is
built from the **released** file, so **none of the six is present in Hubot**, and
none of the six is fixed by Hubot either. They are simply not this package's
subject.

**FIT**: Hubot's one behavioural change targets a defect that IS in the release —
the abort. **GAP**: everything else on that thread remains upstream's, unfixed and
under a Chair-ordered hold.

## 5. Instruments armed today, outside the package

| specification | state | FIT / GAP |
|---|---|---|
| measure defects found at Vision 15's worst rung | `downstream_catch()`, 8/8 two-sided, 2 mutants fail | **FIT** — first live number: 22 findings across 5 items, 4 answered inline |
| ratio of ours-caught vs theirs-caught | not measurable from the API | ⚑ **GAP** — numerator only, and the report says so |
| refuse self-proclaimed labels in maintainer-facing text | armed in the live gate, 8/8, quoted text exempt | **FIT** |
| a human's judgment arriving in the human's own voice | **no instrument, deliberately** — a gate for it is satisfiable by the pen | **GAP, and it is meant to stay one** |
| ruling banner reports only current rulings | fixed, 5/5, mutant fails | **FIT** |

## 6. The honest summary in one line

**Design: covered. Code: written. Build: never attempted, and the one measurement I
did on the committed tree found that it would not have succeeded.** Everything in
column FIT above is a claim about source, not about behaviour, until FBR builds it.
