# hubot — SOTIF performance-insufficiency analysis and assumptions of use

**Component:** `hubot::ZoneParameterFilter` · **Standard frame:** ISO 21448 (SOTIF)
**Class:** QM / advisory / information-only · **Authored:** 2026-09-06 by FSE
**Audited by:** AAA + DIA — *this document is a producer's output and has not been audited yet.*
**Revised 2026-09-06 (second pass).** ⚑ **Every line citation here was re-derived from its anchor
FOUR TIMES during the turn that wrote this line, because the file it cites was rewritten underneath
it four times** (`src/…cpp` at 18:42:44, 18:50:28, 18:54:12; `…hpp` at 18:41:58 and 18:47:00).
**They are true against the tree as it stood at 18:54:12 on 2026-09-06, and a re-derivation is OWED immediately before publish —
see §6, where the countermeasure is named.** ⚑ **No commit SHA is cited anywhere in this document, deliberately: the trailer strip planned before publish rewrites most of this history, so a SHA here would be stale by construction — a lesson this document learned the same afternoon, when the SHA it first cited was rewritten twenty minutes later.** Each carries its grep anchor so a reader whose tree has moved can re-find
it without us; **PI-1 / PI-2 / PI-4 CLOSED**, PI-1 and PI-2 by execution this turn; **§4A and the
§5 ruling are NEW and are OPEN pending DIA** — this seat may not certify its own self-correction
(OPS-064(C), bypass: none).

---

## 0 · The ceiling, stated before anything else

**No ASIL is claimed, rated, decomposed or implied anywhere in this document.** There is no item,
no actuator and no vehicle-level hazard analysis here, and none is performed: a reusable library
cannot perform an integrator's vehicle-level HARA, and claiming to would be the over-claim this
seat exists without. ISO 26262 ASIL / FFI / FMEDA / safety-goals / FSC-TSC are **literacy only**
and are not applied to this component.

**What is claimed:** this component reports, and its report can mislead while every line of it
executes exactly as written. That is a SOTIF question and this document answers only that.

> ### ⚑ AND A BOUND THAT SITS ABOVE ALL OF IT — **NOBODY HOLDS THIS PACKAGE.**
>
> **Vision 45** — *invention that does not reach the market is not yet complete; commerce is how
> service lands on the user.* **No released `nav2_costmap_2d` carries the `ZONE_PARAMETER_FILTER`
> constant this package's source names, so no integrator can install it today.** The package says so
> on its own front page, above its build instructions.
>
> **That does not make the analysis wrong. It makes it INCOMPLETE BY CONSTRUCTION, and the act that
> completes it is not ours** — it is an upstream release. **Whatever this document establishes, it
> establishes about something no person yet holds.** Every performance insufficiency below is
> therefore a **latent** finding twice over: latent because this component actuates nothing, and
> latent because nobody is running it.
>
> The honest consequence: **the assumptions of use in §5 have never been accepted by an integrator,
> because there is no integrator.** They are written for a reader who does not exist yet. That is a
> reason to keep them exact, not a reason to relax them.

---

## 1 · Why ISO 21448 and not ISO 26262 — ruled, not inherited

The governing standard for this component is **ISO 21448, not ISO 26262**, and the reason is
specific rather than definitional.

ISO 26262 governs hazards arising from **malfunctioning behaviour** — the function fails. Every
defect in this component's four-generation history is the opposite: **the function succeeded.**
`enforcementToken()` computed correctly from its inputs on all four occasions. Nothing threw,
nothing returned an error, no memory was corrupted, no deadline in the software sense was missed.
The hazard was that **a correct computation over incomplete observation is indistinguishable, on
the wire, from a correct computation over complete observation.**

That is ISO 21448's subject exactly: a **performance insufficiency** of the intended function,
activated by a **triggering condition**, producing hazardous behaviour with no fault present.

⚑ **One correction to the framing, which strengthens it rather than weakening it.** This component
issues no control action, so it cannot itself produce a hazardous *manoeuvre*. Its output is an
input to a human's decision and, through the integrator's own gating code, to a robot's. **The
SOTIF analysis therefore terminates at the boundary of this component and continues in the
integrator's system**, which is why §5's assumptions of use are load-bearing rather than
advisory. A performance insufficiency here is a **latent** contribution; whether it becomes
hazardous behaviour is decided by code we do not write. **We are responsible for the honesty of
the number and for saying, precisely, what it does not cover.**

---

## 2 · The intended functionality, stated so an insufficiency can be measured against it

> Inside a mapped zone, apply the configured parameter limits to the configured target nodes, and
> **tell a person, in words, whether those limits actually took** — before the *enforcement
> outcome* is known, as an offer, with the person keeping the decision.

⚑ **This statement said "before the risk arrives" until 2026-09-06, and the correction is
load-bearing for every row below, because this is the yardstick each insufficiency is measured
against.** No lookahead exists in this component: `process()` is handed one
`geometry_msgs::msg::Pose` — the robot's current pose — and there is no plan, path, trajectory or
velocity in the translation unit (`src/zone_parameter_filter.cpp:552` (`process()`)). The window this
component actually occupies is between *the limit being requested* and *the answer being known*;
it is not the window between now and a hazard. **Two consequences, and both cut:** an
insufficiency may not be raised here against hazard-anticipation, because the component does not
claim it; and no row below may be read as evidence that hazard-anticipation is covered.

Two properties of that sentence do the work in everything below:

1. **The report's subject is a claim about another process's state** — a target node's live
   parameter value — which this component observes only indirectly, only on request, and only
   while something is driving it.
2. **The report is consumed as permission.** The documented consumer rule is a whitelist:
   *proceed only on `enforced: yes`*. So the reassuring value is the one that moves a robot.

---

## 3 · ⚑ THE CLASS — one insufficiency, five generations closed and a sixth channel unwatched

Four generations were fixed as separate bugs. They are one insufficiency, and naming it is the
whole point of this document, because **the fifth generation is already in the source and a fifth
audit is not what should find the sixth.**

> ### PI-CLASS — The reassuring value is computed from the ABSENCE OF NEGATIVE EVIDENCE, in a
> ### component where absence of negative evidence is produced both by *"all is well"* and by
> ### *"nothing is looking."*
>
> `enforced: yes` does not mean *confirmation was observed*. It means *no disconfirmation is
> currently recorded*. Those coincide **only while the mechanism that could disconfirm is live.**
> Every generation of this defect is one further way for that mechanism to be non-live while the
> token still computes to the reassuring value.

| gen | triggering condition | the observer that was not live | status |
|---|---|---|---|
| **1** | sets ISSUED, none answered | the answer had not arrived | CLOSED — `unconfirmed_targets_` → `pending` |
| **2** | filter configured, nothing applied | no state had been applied | CLOSED — `state_initialized_` gate |
| **3** | costmap stopped calling `process()` | the result check ran only from `process()` | CLOSED — liveness timer + `unknown` |
| **4** | before the first tick, every ordinary bringup | `livenessTick()` was a new caller | CLOSED — `notWatching()`, in the VALUE |
| **⚑ 5** | **the liveness gate is configured inert** | **`costmap_silent_` can never become true** | ⛑ **CLOSED 2026-09-06 — fallback budget `liveness_period x kValidForPeriods` (`cpp:1133-1136`); SC-1 + SC-2 GREEN, executed this turn** |
| **6?** | *unknown* | *unknown* | ⚑ **The class is not closed by closing generation 5.** Five of five went into a caller or an input; the invariant SOTIF-CLASS-1 is what holds, and only `FSE-SC-1..3` would extend it to the SENTENCE channel (§4A). **They are specified and not written.** |

**The package's own diagnosis of generations 1–4 is correct and is quoted here because it is the
reason generation 5 exists** (`include/hubot/zone_parameter_filter.hpp:406`):

> *"Every fix went into a caller and every new caller arrived without it. So this one goes into
> the VALUE."*

⚑ **The gate moved into the value. Its INPUTS did not.** `notWatching()` reads `costmap_silent_`,
which is computed at `src/zone_parameter_filter.cpp:1132-1136` (`silence_budget`) from a **fallback** budget. **A gate could not report its own inertness, and this one did not.**
⛑ **FIXED 2026-09-06 in the INPUT rather than the gate** — and the record of *how* is the load-
bearing part: `CHANGELOG.md:217` shows the fix this document's own SC-1 comment proposed
(a `costmap_silence_timeout_ <= 0` disjunct in `notWatching()`) was **implemented and refuted** —
it turned SC-1 green and **SC-2 red**, buying the oracle by making the filter never say `yes` at
all. **A constant disjunct cannot tell a driven costmap from a stopped one.** The gate was always
right; its input was what could not move. **Generations 6 and 7 will still come from the same
place unless the invariant, not the instance, is held** — and §4A is the first evidence that the
next one may come from the sentence channel, where no invariant is held at all.

> **⚑ SOTIF-CLASS-1 — the invariant that makes this a class rather than a list.**
> **No configuration of the liveness gate may produce a reassuring value from a filter that
> nothing is driving.** A detector an integrator switched off must report that it is off, **in the
> value**. Suppressing a warning and asserting safety are different acts.
>
> Executable form: `test/sotif_gate_inertness_test.cpp`, SC-1 and SC-2.

---

## 4 · Performance-insufficiency table

Ranked by how directly the insufficiency reaches a consumer acting on `enforced: yes`.

| # | performance insufficiency | triggering condition | resulting output | why it misleads | evidence (read 2026-09-06) | verdict |
|---|---|---|---|---|---|---|
| **PI-1** | ⛑ **CLOSED 2026-09-06 BY A CHANGE TO THE CODE, AND THIS ROW SAID OTHERWISE FOR PART OF A DAY.** *The insufficiency was: the silence detector could be configured inert, and the component then made an affirmative liveness claim instead of withholding one.* | `costmap_silence_timeout <= 0` **and** the costmap subsequently stops. | **On the tree as it stands: `watching: NO`, `enforced: unknown`, level `STALE`.** | It no longer misleads. `livenessTick()` now falls back to a budget of `liveness_period x kValidForPeriods` when the explicit one is switched off, so `costmap_silent_` is reachable in every configuration that has a heartbeat at all. **Disabling a detector removes a measurement; the code no longer lets it manufacture a good one.** | `cpp:1133-1136` (`silence_budget`, the fallback) · `cpp:1137` (`silent` derives from it) · `hpp:436` (`notWatching()`) · `cpp:1374` (`add("watching", …)`) · `README.md:537` · **executed this turn — see below** | ⛑ **CLOSED — CONFIRMED BY EXECUTION 2026-09-06 (SC-1 GREEN, SC-2 GREEN). RESIDUAL BELOW, AND IT IS ONE THING.** |
| **PI-2** | ⛑ **CLOSED 2026-09-06 BY A CHANGE TO THE CODE.** *The insufficiency was: the declared expiry exceeded the publisher's own threshold for calling itself stale, and neither number had a ceiling.* | Shipped defaults. No misconfiguration required. | **On the tree as it stands: `valid_for_s: 2.0` at shipped defaults — clamped, not `2.5`.** | `declaredValidForS()` now returns `min(liveness_period x kValidForPeriods, costmap_silence_timeout)` while that detector is on, and the same number is offered as the DEADLINE, so the promise on the QoS channel and the promise in the payload are one promise computed in one place. The unbounded widening is gone with it: `liveness_period: 30.0` no longer yields `valid_for_s: 75.0` unless the silence budget genuinely permits it. **A warning fires once at configure naming both numbers, so the clamp is visible rather than silent** — observed in this turn's own run. | `cpp:1073-1091` (`declaredValidForS()`, the clamp) · `cpp:61` (`kValidForPeriods = 2.5`) · `cpp:174` (the clamp warning) · `hpp:595` · `README.md:343` | ⛑ **CLOSED — CONFIRMED BY EXECUTION 2026-09-06 (SC-3 GREEN)** |
| **PI-3** | **`valid_for_s` bounds the freshness of the REPORT; it does not bound the validity of the CLAIM, and the two are presented as one number.** | Any third party sets the same parameter on a target after this component's confirmation. | A message inside its `valid_for_s` window carrying `enforced: yes` for a value that is no longer set. | The component's own AoU-4 already states that a confirmed set means *acceptance*, not *still holding* — **unbounded in time, unverified by construction.** So the claim can go false at any instant, with no bound at all, while the only number a consumer is given to reason about expiry is a **publisher-liveness** number. A consumer told *"believe it for `valid_for_s`"* will infer the converse. The converse is not warranted. | `hpp:718` (AoU-4) · `cpp:210-219` (`decision_qos`'s own rationale, which is about node death only) | ⛑ **OPEN — inherent, and RE-RULED 2026-09-06. This cell read *"disclosure is the countermeasure"*, which §5 now rules is not a countermeasure at all. AoU-SILENT: an unmitigated residual, offered for transfer, accepted by nobody.** |
| **PI-4** | ⛑ **CLOSED 2026-09-06 BY A CHANGE TO THE CODE.** *The insufficiency was: the only complete countermeasure the component recommended was structurally incompatible with the component's own publisher.* | An integrator follows `README.md:557` and requests a finite `DEADLINE` QoS on their `zone_decision` subscription. | **On the tree as it stands the subscription matches**, because the publisher now offers a finite deadline. | `decision_pub_` is created with `decision_qos.deadline(declaredValidForS())` whenever `liveness_period > 0`, so the offered deadline is finite and equal to the published `valid_for_s`. The bare-`rclcpp::QoS(10)` state survives only as the record of what it was. ⚑ **And the honest half: with `liveness_period <= 0` no deadline is offered, because no rate could be kept — the README says so and an integrator must not request one in that configuration.** | `cpp:232` (`decision_qos.deadline`) · `cpp:202` (the record of the bare QoS) · `cpp:210-219` · `README.md:557-572` · `hpp:757-764` | ⛑ **CLOSED — CONFIRMED BY EXECUTION 2026-09-06 (SC-5 GREEN, both arms). ⚑ BOUND: `rmw_fastrtps_cpp` only.** |
| **PI-5** | **Detection latency is bounded by `liveness_period`, not by the `costmap_silence_timeout` an integrator sets to express their budget.** | `liveness_period > costmap_silence_timeout` — e.g. `0.5` / `5.0`. | Silence declared after up to `costmap_silence_timeout + liveness_period`, not `costmap_silence_timeout`. | The parameter named for the budget does not govern the budget. No consistency check exists between the two; the startup warnings at `cpp:156` and `cpp:174` fire **only on `<= 0`**, so a self-defeating pair is accepted in silence. | `cpp:1131-1137` (age evaluated once per tick) · `cpp:156` + `cpp:174` (warning coverage) | **OPEN — LIVE. AoU-SILENT (AoU-S5): nothing on the wire distinguishes a self-defeating pair from a sane one.** |
| **PI-6** | **The `unknown` vocabulary is asserted positively on one of the two branches that produce it.** | A future change alters the never-driven branch's token. | `MF-1` asserts three negatives (`!= yes`, `watching != yes`, `level != OK`); a change emitting `NO` there passes. | The `unknown` / `NO` distinction is load-bearing and the component argues it at `cpp:1245` — STALE not ERROR, because *"nothing was measured and found bad; nothing was measured."* `NO` sends an operator to look for a fault that does not exist. A vocabulary argued this carefully needs a positive oracle on every branch. | `test/maintainer_findings_test.cpp:410-431` (MF-1, negatives) vs `:640-666` (MF-6, positive, stopped branch only) | **CLOSED as a value defect — SC-4 GREEN; the branch was unasserted, not wrong. Oracle now exists.** |
| **PI-7** | **State 0 reports nominal defaults *in force* having issued nothing, when `nominal_defaults` is empty.** | `nominal_defaults` declared empty; robot leaves the mask. | level `OK`, *"Outside any zone; nominal defaults are in force."* — zero sets issued, zero confirmed. | A positive claim about a restoration that did not occur. **Partially mitigated:** a per-parameter config-load warning fires when a state overrides a parameter with no nominal entry (`cpp:497`, `!has_nominal`). It does not fire for the wholly-empty case. | `cpp:816` (`resetToNominal()` iterates an empty map) · `cpp:1332` (the claim) | ⛑ **OPEN — RE-RULED 2026-09-06. This cell read *"disclosure sufficient"*. It is not: §5 rules prose is not mitigation. AoU-SILENT. ⚑ The `low reach` half stands and was re-checked — probed live this turn with a NON-empty `nominal_defaults`, the resets are genuinely issued and confirmed (`pending` → `yes`), so the trigger really is the wholly-empty case.** |
| **PI-8** | ⛑ **RE-RULED 2026-09-06: THE INSTANCE IS CLOSED, THE CLASS IS NARROWED, AND THE HALF THAT ACTUALLY HURT A PERSON WAS THE SENTENCE.** *The insufficiency was: the report reached the right VERDICT by the wrong REASON when a target was named relatively, and sent an operator to investigate a healthy node.* | `node:` has no leading `/` and the costmap node is namespaced — which nav2 always does in production. | `enforced: NO` with a reason that named a node which was reachable the whole time. | **The instance:** the declared name is now passed through `joinWithParentNamespace()` (`cpp:387`), so a relative `node:` under `/local_costmap/local_costmap` resolves to the node that was reachable all along. **The class:** ⚑ **the verdict was never the defect — the SENTENCE was**, and *"no answer within Ns"* described **both** a present-but-silent target **and** a name that addresses nothing, which are opposite instructions to a person at 03:00: *walk to that node* versus *fix that line of YAML*. `describeUnansweredTarget()` (`cpp:861`) now says which it observed, carries **both spellings** of the name where the join moved it (`cpp:394-398`), and ⚑ **refuses to assert absence** — discovery is asynchronous, so it reports what it saw rather than declaring a live node dead, **which would be this same defect mirrored.** | `cpp:387` (the join) · `cpp:861` (`describeUnansweredTarget`) · `cpp:963` (the sentence) · `test/namespaced_target_resolution_test.cpp` (4 cases; C and D a discriminating pair) · `prose_matches_tree.py` SC-11 | ⛑ **CLOSED as the reported instance; the CLASS is covered for the first time by SC-11. ⚑ NOT EXECUTED BY ITS AUTHOR — see §6.** |
| **PI-9** | **A filter can be loaded, configured, activated and driven ZERO times inside a stack that reports itself healthy.** | No layer in the costmap declares update bounds. `CostmapFilter::updateBounds()` ignores all four bounds arguments by design. | `LayeredCostmap` computes an inverted window (`Updating area x: [9, 1]`) and the `if (xn >= x0 && yn >= y0)` guard around every `updateCosts()` never fires. Configure/activate return 0, the update loop runs at its configured rate, the footprint publishes. | Every ordinary health signal is green while the filter has never executed. | ⚑ **AND HUBOT CAUGHT IT** — from inside the live stack, unprompted: *"THIS FILTER IS NOT WATCHING. It has not been driven even once."* | ⚑ **NOT A DEFECT IN THIS COMPONENT — evidence FOR it.** See below. |
| **PI-16** | ⚑ **THE FIX FOR PI-8 IS A SILENT BEHAVIOUR CHANGE FOR THE OPPOSITE TOPOLOGY, AND IT IS THE SAME CLASS ARRIVING FROM THE OTHER SIDE.** | A relative `node:` naming a target that lives **inside** the costmap's own namespace — the one topology that WORKED before the join. | The set is issued at a name nothing serves; the operator gets `enforced: NO`. | **Derived from nav2's own source, read this turn at `1.5.1`:** `joinWithParentNamespace` computes `parent = ns.substr(0, ns.rfind("/"))` and returns `parent + "/" + topic` (`layer.cpp:90-96`). For a costmap node whose namespace is `/local_costmap`, `rfind("/")` is `0`, the parent is the **empty string**, and a relative `foo` becomes **`/foo`**. Before the join the raw name went to `AsyncParametersClient`, where `rcl_node_resolve_name` expanded it against the **owning** node, giving **`/local_costmap/foo`**. ⚑ **So `node: foo` addressing a target genuinely at `/local_costmap/foo` reached it before and does not now.** The change is correct for nav2's conventional topology (servers at the peer level) and wrong for this one, and **nothing announces which it did.** | `cpp:387` (the join) · nav2 `layer.cpp:90-96` at tag `1.5.1`, read from the image this turn · ⚑ **introduced by the commit that ADDED the join, NOT by the later one that added `describeUnansweredTarget()` — two separate changes, and a reader bisecting needs that** | ⚑ **OPEN — CONFIRMED BY SOURCE, NOT EXECUTED. No integrator is broken today because nobody holds the package (§0); after publish that sentence stops being available.** |
| **PI-17** | ⚑ **THE GATE THAT EXISTS TO STOP A RELOAD CARRYING THE OLD CONFIGURATION GUARDS ONE OF THE SEVEN CONTAINERS THAT RELOAD CLEARS.** | Any future edit that drops a `.clear()`. | A reload silently carries the previous configuration's state — which is not hypothetical: the header records that `state_param_map_`, `nominal_defaults_` and `param_clients_` **once had no `.clear()` anywhere in the translation unit**, so every reload duplicated every nominal default without bound and kept clients for targets the new configuration had dropped. | **The code is correct today** — measured this turn, `resetFilter()` clears **seven** containers (`cpp:1489-1497`). **INV-D derives its member list as `grep -oE '\bpending[A-Za-z0-9_]*_\b'` over the header** (`test/safety_invariants_static.sh:217`), so it evaluates **`pending_sets_` and nothing else**. Delete `nominal_defaults_.clear()` tomorrow and INV-D stays GREEN. ⚑ **And the seventh container is `declared_target_names_`, added the same day to stop the component naming the wrong node — a stale entry there makes it name the wrong YAML LINE, which is that defect re-entering through the reload door, past its own gate.** The invariant's TEXT is right (*"a reload must not carry the previous configuration's results"*); its DERIVATION is narrower than its text, and a check named for a property but keyed on a variable prefix is the same defect INV-D's own comment records CPP fixing in it on 2026-09-05. | `cpp:1489-1497` (the seven clears) · `test/safety_invariants_static.sh:217` (the one-prefix derivation) · `cpp:1455-1457` (the header's record of the original defect) | ⚑ **OPEN — the CODE is green, the GATE is narrow. Routed to this seat by CPP on its own gate rather than changed quietly; `test/` is CPP's to build.** |

> ### ⛑ RULED 2026-09-06 BY FSE, WHO OWNS THESE CELLS AND WHOSE CELLS WERE THE WRONG ONES.
>
> **CPP raised that three documents disagreed about PI-2 and PI-4, repaired the citations, and
> deliberately left the verdict cells untouched because a safety-class disposition is not its to
> make. That was the correct call and it is why this ruling exists.** A build seat that had
> repaired a verdict it was confident about would have been right on the facts and wrong on the
> boundary, and the next such repair would have been wrong on both.
>
> **THE RULING: PI-1, PI-2 and PI-4 are CLOSED. The code moved and this table did not.** Ruled from
> a re-measurement of the configuration path performed by this seat this turn — not inherited from
> CPP's reading, not from the pen's, and not from the CHANGELOG.
>
> ⚑ **AND THE VERDICTS WERE NOT CLOSED ON A SOURCE READ, BECAUSE CLOSING IS THE DIRECTION THAT
> REACHES HER.** An open row that should be closed over-warns an integrator; a closed row that
> should be open under-warns one, and only the second reaches a person in the form of a robot that
> was believed. **The warrant required for a closure is therefore strictly higher than the warrant
> that opened it, and a source read is not it.** Built and run this turn, image
> `nav2-lyrical-main-compat:latest`, `--network=none`, ROS `lyrical`, `rmw_fastrtps_cpp`, build
> **exit 0 in 32.9 s**:
>
> | case | result this turn | what it establishes |
> |---|---|---|
> | **SC-1** | ⛑ **GREEN** (was RED) | a stopped costmap with the detector disabled no longer reads `watching: yes` — **PI-1 closed** |
> | **SC-2** | ⛑ **GREEN** | the negative control holds, so SC-1's green is **not** the degenerate pass of a filter that never says `yes`. ⚑ **This pair is the whole ruling.** `CHANGELOG.md:217` records that the fix this document's own SC-1 comment proposed turned SC-1 green and **SC-2 red** — buying the oracle by making the component useless. Both green means the fix went into the input, not into the gate |
> | **SC-3** | ⛑ **GREEN** (was RED) | `valid_for_s` is clamped — **PI-2 closed** |
>
> Runtime evidence, from this turn's own log rather than from a claim about it — the fallback
> announcing itself at configure with `liveness_period: 0.1`:
>
> ```
> [WARN] ZoneParameterFilter: costmap_silence_timeout is 0.000 (<= 0). The explicit
>        stopped-costmap report is off; the heartbeat keeps publishing and falls back to a
>        budget of 0.250s derived from liveness_period, because suppressing a warning
>        cannot license claiming the costmap is running.
> ```
>
> **PI-4 is ruled CLOSED on the source read plus the package's own executed SC-5**, and is the one
> of the three this seat did not re-execute: its oracle needs the two-arm QoS harness and the
> result is `rmw`-specific either way. **Stated as what it is — the weakest of the three closures —
> rather than levelled up to match them.**
>
> #### ⛑ THE RESIDUAL ON PI-1, STATED EXACTLY AND NOT ONE WORD FURTHER
>
> **The node dies outright.** Then the timer dies with it, nothing publishes, and the last message
> stands. That is the whole residual and nothing running inside a process can close it. It is
> **not** a configuration a person can fall into by accident, it is **not** reachable by any value
> of `costmap_silence_timeout`, and it is already carried by AoU-S6 and by the offered `DEADLINE`.
>
> ⚑ **What is NOT the residual, checked rather than assumed, because a residual that quietly grows
> is how a closed row goes false:** `liveness_period <= 0` disables the heartbeat, and this seat
> tested whether it re-opens PI-1 by another door. **It does not.** No timer is created
> (`cpp:251`), so nothing periodic is published at all — and `declaredValidForS()` returns `0.0`,
> so every message declares itself expired on arrival and no deadline is offered. The
> configuration degrades to *"publishes only on change, promises nothing"*, which is honest.
> **It is a worse configuration and it is not a misleading one.**
>
> ⚑ **And one prediction of this seat's own was REFUTED by running it, recorded because a
> refutation is a measurement and hiding it would leave the rest unwarranted.** This seat reasoned
> from source that a sixth generation existed: `state_initialized_` gates only the change-publish
> path (`cpp:1176` (`if (!state_initialized_)`)), while `livenessTick()` calls `publishDecision()` directly, so a filter
> driven before any state was applied should have heartbeat `enforced: yes` while `configured: not
> yet`. **Probed on the live fixture with a zero-filled mask: `configured=yes` on every sample.**
> Entering state 0 *is* entering a state, and it initialises. **The hypothesis is dead.**

> ### ⚑ PI-9 IS THE ONE ROW IN THIS TABLE THAT IS GOOD NEWS, AND IT SHOULD BE READ AS SUCH.
>
> The generation-4 fix — moving the gate **into the value**, so `enforcementToken()` and `watching`
> both derive from `notWatching()` and no caller can compute around it — **caught a real defect
> nobody planted, in a real navigation stack, that the stack's own health signals did not surface.**
> That is **Vision 88** performing: *the production system self-reports when it has departed from
> contract, so humans investigate rather than stand watch.*
>
> ⚑ **And the obvious fear is refuted, measured rather than assumed.** One would expect PI-1 to
> compose with PI-9 into something worse — disable the silence detector, never drive the filter,
> and get `enforced: yes` from something that has never run. **It does not compose.**
> `notWatching()` is `!ever_processed_ || costmap_silent_` (`hpp:436`), and **the first disjunct is
> independent of `costmap_silence_timeout`.** A never-driven filter reports `unknown` even with the
> detector switched off. SC-4 covers exactly this and is **GREEN**.
>
> **The value-level gate is the reason PI-1 is survivable. Fixing PI-1 in a caller would have
> re-opened this.**

### Closed — and ⚑ **"closed" was measured, 2026-09-06, and TWO OF SIX HAVE NO ORACLE**

⚑ **This table said "closed by `<code citation>`" for all six and cited no test for any of them.**
Closed by a **code change** is not closed by a **verified guard**: it records the absence of a known
defect as evidence the defect is gone, without anything that would notice its return. **That is
PI-CLASS, committed inside the table that defines PI-CLASS**, and it is **Vision 9** — *the operator
must not be the last line of defense against defects; the machine itself must catch them.*

**Coverage measured by grep over `test/` this turn, not recalled:**

| # | insufficiency | closed by (code) | ⚑ oracle that would catch its return |
|---|---|---|---|
| PI-C1 | `enforced: yes` on issue | `unconfirmed_targets_` → `pending` (`cpp:895`) | **COVERED** — `degrade_at_production_caller_test.cpp:870` (CPP_N3), `maintainer_findings_test.cpp:611`, `costmap_silence_liveness_test.cpp:518` |
| **PI-C2** | `enforced: yes` at startup | `state_initialized_` gate (`cpp:1176`) | ⛑ **RE-RULED 2026-09-06 — MIS-ATTRIBUTED, not merely uncovered. This cell reached the right conclusion by the wrong route.** Measured this turn: the `state_initialized_` check sits in `publishDecisionOnStatusChange()` and suppresses only the **change-triggered** publish; `livenessTick()` calls `publishDecision()` **directly**, past it. **So it is not what stops `enforced: yes` at startup on the heartbeat path** — `notWatching()`'s `!ever_processed_` disjunct is, and that IS covered (PI-C4 / MF-1). ⚑ **PI-C2 and PI-C4 are the same guard, counted twice, scored once covered and once not.** ⚑ **And this seat predicted a live sixth generation from that gap and PROBED it: with a zero-filled mask the heartbeat reported `configured=yes` on every sample — entering state 0 *is* entering a state. REFUTED. The residual is a bookkeeping error in this table, not a defect in the component.** |
| PI-C3 | `enforced: yes` during silence | liveness timer + `unknown` (`cpp:251` (timer) · `cpp:1129-1161` (`livenessTick()`)) | **COVERED** — `costmap_silence_liveness_test.cpp` CASE 1, with a negative control that must reproduce the defect |
| PI-C4 | `enforced: yes` before the first tick | `notWatching()` in the value (`hpp:436`) | **COVERED** — `maintainer_findings_test.cpp:410` (MF-1) + MF-2 negative control |
| **PI-C5** | silent unenforcement when no client exists | `markTargetDegraded` on missing client (`cpp:900` (`"no parameter client exists"`)) | ⚑ **UNCOVERED.** No test configures a target for which no client was built. The `target_exists=false` arm in `costmap_silence_liveness_test.cpp:239` routes at a node that is not running — **a client IS built and the service never answers**, which is PI-C6, a different door. |
| PI-C6 | a set that never answers reported as success | `set_parameters_timeout` deadline (`cpp:954` (`> set_parameters_timeout_`)) | **COVERED** — `degrade_at_production_caller_test.cpp:742` (`FSE_F_ATargetThatNeverAnswers…`) |

⛑ **RESOLVED 2026-09-06: the full suite was re-run against the current tree this turn — 52 tests, 0 failures, 0 skipped (see §6). The staleness described below is closed; the paragraph is kept because it is the reasoning that caught it.**

⚑ **And the closed set was closed against a codebase that has since moved.** In the build workspace
used for this analysis the newest `colcon` test-result directory is **`test-result_2026-09-05_23-34-16`**,
while the binaries are dated **2026-09-06 08:32** and the source gained an entire new suite
(`maintainer_findings_test.cpp`) plus the `notWatching()` fix at **09:55 the same morning**. **That
workspace's record shows no suite run after those changes.** A closed entry that has not been
re-tested since the code moved is the same shape as a green suite that never runs.

⛑ **PI-C2 is re-ruled above: guarded, by a different mechanism than this table named. PI-C5 is genuinely unguarded**, and it belongs on the open side
until an oracle exists. Neither is expensive: both are single cases in the existing fixture.

---

## 4A · ⛑ THE SENTENCE A PERSON READS — the half of the bridge this analysis never opened

> **This section exists because this document did not have it, and the omission is this seat's,
> named by this seat against itself before anyone audited it.** It is authored here; **it is not
> closed here.** DIA rules its scope and whether the document now owes more. **FSE may not certify
> its own closure (OPS-064(C), bypass: none) and does not attempt to.**

**The measurement.** `"Decide as if"` appears in this document **zero** times. So does `st.message`.
So does *summary*, *human-readable*, and *sentence-as-output*. Every row of §4 is written about a
**key/value pair** — `enforced`, `watching`, `level`, `valid_for_s`, `costmap_age_s`, `report_seq`.
**Not one row is about the natural-language sentence**, and the sentence is the only part of a
`DiagnosticStatus` that a human being actually reads in `rqt_robot_monitor` or a terminal.

⚑ **Why that is not a gap in coverage but an inversion of the thesis.** The governing thesis of
this package is `D-VGC264-1` — *hubot is a bridge between robot output and human sensor.* This
analysis opened the **robot-facing** half of the bridge and never opened the **human-facing** half.
The human sensor is the half the thesis is named for.

⚑ **And it is not hypothetical, which is what makes it a finding rather than a scruple. THREE
DEFECTS HAVE ALREADY BEEN FOUND AND FIXED IN THOSE SENTENCES, EACH MARKED `⛑` IN THE SOURCE, AND
NOT ONE OF THEM APPEARS ANYWHERE IN THIS TABLE:**

| # | the defect in the sentence | why it mattered | where the record is |
|---|---|---|---|
| **s-1** | Both branches opened with a bare *"NOT WATCHING."* | At `STALE` level beside a moving robot, **the subject a reader supplies for a verb with none is the ROBOT.** | `cpp:1264-1267` (`"Both opened with a bare"`) |
| **s-2** | *"…because nobody is"* / *"…and nobody is watching"* | The filter measured **one** thing — that *it* has not been driven — and asserted a fact about the safety scanner, the bumper, the E-stop and the person in the doorway. ⚑ **A failure-shaped value overstating danger: the exact inverse of the success-shaped value this package exists to abolish, and nothing in the corpus was looking for that sign.** | `cpp:1268-1277` (`"because nobody is"`) |
| **s-3** | *"Do not rely on its limits until this reads yes"* | An instruction to wait **with no end on it**, published beside a robot that is moving. There is an end: `set_parameters_timeout`. | `cpp:1302-1305` (`"BOUND THE WAIT"`) |

**Three defects, a demonstrated defect rate, and this instrument returned zero from that surface.**
That is this corpus's own test of an instrument — *if the method could not have surfaced a
counter-example, it has measured nothing* — and this method could not, because its unit of
analysis was the key/value pair.

### The rows the sentence channel is owed

**Ranked, as §4 is, by how directly the insufficiency reaches a person acting on what it says.**

| # | performance insufficiency | triggering condition | why it misleads | evidence (measured 2026-09-06) | verdict |
|---|---|---|---|---|---|
| **PI-10** | ⛑ **THE STARTUP LOG ANNOUNCES A SILENCE BUDGET THAT IS NOT THE ONE IN FORCE, AND THE CORRECT REASONING IS IN THE SAME FILE 825 LINES AWAY.** | `costmap_silence_timeout <= 0`. Fires at **every** configure in that configuration. | The `INFO` at `cpp:255-258` (`"is reported as stopped after"`) prints the **configured** value: *"the costmap is reported as stopped after **0.000s**"*. The effective budget is `liveness_period x 2.5`. ⚑ **The `ERROR` at `cpp:1142-1148` (`"no costmap update for"`) prints the EFFECTIVE budget and its own comment says exactly why** — *"printing it would tell an operator the limit they just tripped was zero seconds."* **The same reasoning was applied at one site and not the other, in one file, on one day.** An operator reads *0.000s* at bringup and a real number under fault, and cannot reconcile them. | ⚑ **OBSERVED IN THIS TURN'S OWN RUN**, three lines apart in one log: the `WARN` says *"falls back to a budget of 0.250s"*, the `INFO` below it says *"reported as stopped after 0.000s"* | ⚑ **OPEN — CONFIRMED BY EXECUTION 2026-09-06. `src/` is CPP's; named, not edited.** |
| **PI-11** | **The sentence channel carries imperatives the value channel does not, and no assumption of use bounds them.** | Every `STALE` report. | Both never-driven and stopped-costmap sentences end *"Decide as if this zone is unchecked"* / *"Decide as if no zone limits are being applied."* **These are the only outputs of this component that tell a person what to do**, and §5 governs only what the *tokens* mean. An integrator can honour every assumption of use in this document and still ship a screen whose imperative fires through ordinary bringup. | `cpp:1283` and `cpp:1296` (`"Decide as if"`, both branches) · the package's own `test/prose_matches_tree.py` lists `"Decide as if"` under `IMPERATIVE_MARKERS` and a retired variant under `RETIRED_PHRASES` — **the code knows this class exists and this document did not** | **OPEN — the sentence is unbounded by any AoU. See AoU-S8.** |
| **PI-12** | **The sentence and the token can disagree, and nothing asserts that they agree.** | Any future edit to either. | The token is computed in `enforcementToken()` (`cpp:1037` (`enforcementToken()`)); the sentence is composed in `publishDecision()` (`cpp:1183` (`publishDecision()`)) from an overlapping but **separate** set of conditions. They are consistent today by the author's care, not by construction — **which is precisely the property `PI-C1..PI-C6` were re-measured for, applied to the other field.** | no test in `test/` asserts a relation between `st.message` and `st.values`; measured by grep this turn | **OPEN — unguarded.** |
| **PI-13** | **A consumer following this document's own whitelist rule never reads the sentence, and the person does.** | Always. | **AoU-S2** says *proceed only on `enforced: yes`* — a rule stated over the **token**. A correct integrator therefore gates on the token and renders the sentence to a human. **So the field with no oracle is the field the human reads, and the field with six oracles is the one the robot reads.** The coverage is inverted with respect to `D-VGC264-1`. | AoU-S2, this document · the six `PI-C*` oracles are all token-side | **OPEN — inherent to the split; disclosure is NOT sufficient, see §5A.** |
| **PI-14** | **The sentence is the one surface on which this component can assert about things it cannot see.** | A future edit that adds a clause. | Defect **s-2** above was exactly this and shipped. A token drawn from a closed four-value vocabulary **cannot** over-claim about the E-stop; an English sentence can, and did. ⚑ **The vocabulary discipline that protects the token has no counterpart on the sentence.** | `cpp:1268-1277` (`"because nobody is"`) (the record of s-2) | **OPEN — no closed vocabulary, no oracle.** |
| **PI-15** | ⚑ **An imperative that fires when nothing is wrong teaches an operator to discount the one that fires when something is.** | The never-driven branch publishes at 1 Hz **from configure, before the costmap update thread exists** — so through every ordinary bringup. | The source records this as `⛑ FSE-W1` and **differentiated the two sentences' standing in response**, which is a real mitigation and is why this row is not ranked higher. **It is not measured.** Nothing establishes how long the never-driven sentence persists on a real bringup, or that an operator distinguishes the two. **The cry-wolf risk was reasoned about and reduced; it was never observed.** | `cpp:1285-1292` (the `⛑ FSE-W1` record) · no measurement exists | **OPEN — mitigated in the sentence, UNVERIFIED in the field.** |

### The oracles these rows are owed — ⚑ SPECIFIED HERE, NOT WRITTEN, AND THAT IS A DEFECT IN THIS DELIVERABLE

**This seat's charter requires a CODE-PRIMARY deliverable: an executable invariant or test, with the
analysis as the byproduct. This turn produced the byproduct and not the deliverable**, because
`test/` and `src/` are held by another seat for this turn. **Naming that is the duty; it does not
discharge it.** The three below are specified tightly enough to be written by whoever holds the pen
next, and each carries the argument that it is **satisfiable** — because this file's own SC-1 was
once written in a form that could not pass against a correct implementation, and a gate that cannot
pass gets deleted rather than fixed.

- **FSE-SC-1 — the sentence must not contradict the token in the same message.** For every emitted
  report, assert the pairing: a message containing *"NOT WATCHING"* accompanies `watching: NO`, and
  `watching: yes` never accompanies it. **Satisfiable:** both are computed in one function from one
  lock-held snapshot; the case asserts a relation that already holds, so it is a *tripwire*, not a
  defect test, and it goes red only when someone breaks it. **Negative control required:** a build
  with the branches deliberately crossed must make it red, or it proves nothing.
- **FSE-SC-2 — no log line may state a silence budget other than the effective one.** Capture the
  configure-time log with `costmap_silence_timeout: 0` and assert no line reports `0.000s` as the
  stopped-costmap threshold. **Satisfiable and currently RED — this is PI-10's oracle**, and it is
  the one of the three that should be written first because it fails today.
- ⛑ **PARTIALLY LANDED WHILE THIS SECTION WAS BEING WRITTEN, BY ANOTHER SEAT, AND THE
  CONVERGENCE IS THE EVIDENCE THIS SECTION IS RIGHT.** The same commit that added `describeUnansweredTarget()` adds **SC-11**, which
  applies SC-9's rule to the **three failure-sentence functions SC-9 never read** — and its own
  record states that *"the longest and most directive sentence in the package … was written
  outside every oracle in that file on the day it was authored."* **Two seats reached the same
  finding on the same day from opposite directions: this one by asking what the analysis had not
  analysed, that one by asking what the gate had not read.** ⚑ **SC-11 covers WHICH TOKENS a
  sentence may contain. It does not cover FSE-SC-1 (sentence-vs-token agreement), FSE-SC-2
  (PI-10, still open and still red), or whether an imperative is bounded by an assumption of use
  (PI-11).** The gap narrowed; it did not close.

- **FSE-SC-3 — the sentence may assert only what this filter measured.** Assert that no emitted
  `st.message` contains a term from the package's own `FORBIDDEN_TOKENS`, and that every imperative
  it does contain is one §5 names. ⚑ **Bound, stated because the last widened gate in this package
  was nearly unsatisfiable:** `FORBIDDEN_TOKENS` includes ordinary English (`"seconds"`, `"safe"`),
  and the shipped sentences legitimately contain *"5.0s"* and time words. **This case must be
  written against the emitted `st.message` only, never against the prose of the docs, and its
  token list must be the narrow operator-instruction subset — not the whole list.** Written wider,
  it cries wolf, and a gate that cries wolf gets switched off, which is worse than not having it.

---

## 5 · Assumptions of use — what an integrator must accept

> ### ⛑ RULED FIRST, BECAUSE IT GOVERNS EVERY ROW BELOW AND EVERY VERDICT ABOVE.
> ### **AN ASSUMPTION OF USE IS NOT A COUNTERMEASURE. THIS TABLE HAS BEEN COUNTING PROSE AS MITIGATION.**
>
> **Raised by VPM against this document's twelve assumptions of use; ruled here, because the
> strength of this analysis's own evidence is inside this seat's ceiling and nobody else's.**
> VPM's sentence is the one that does not survive contact: ***"a countermeasure that only works if
> someone reads it is not one."***
>
> **THE RULING, and it goes against this document.**
>
> ISO 21448 recognises three responses to a performance insufficiency: **modify the function** so
> the insufficiency shrinks; **restrict the operational conditions** so the triggering condition is
> not met; **improve controllability** so a human can intervene. **Prose addressed to an integrator
> is none of the three.** It does not change what the component computes, it does not prevent any
> configuration, and it reaches no one at runtime.
>
> What an assumption of use actually is — and the SEooC frame (ISO 26262 Pt 10) is precise about
> this — is a **condition on the validity of the safety argument**: *this analysis holds provided
> you accept and implement the following.* **That is a TRANSFER of an unmitigated residual to
> another party's ledger, and a transfer has a load-bearing event: ACCEPTANCE.** Until an
> integrator accepts it, nothing has moved. The residual has not shrunk and it has not moved
> either. **It is an offer of a transfer, outstanding.**
>
> ⚑ **And §0 of this document already recorded that nobody can install this package, so the number
> of integrators who have accepted any assumption below is ZERO.** Every AoU here is an outstanding
> offer with no counterparty. **This document has been scoring twelve outstanding offers as if they
> were twelve mitigations.**
>
> **THE CONVICTION IS IN THIS TABLE'S OWN WORDS, and that is why the ruling is not a scruple.**
> Two verdict cells in §4 read, verbatim:
>
> - **PI-3** — *"OPEN — inherent; **disclosure is the countermeasure**"*
> - **PI-7** — *"OPEN — low reach; **disclosure sufficient**"*
>
> **This seat wrote both.** A verdict cell that names disclosure AS the countermeasure is the exact
> error, stated on the face of the artifact, twice. **Corrected in place below rather than
> reworded, because the words are the evidence.**
>
> #### The distinction this table should have drawn and did not
>
> | class | test | strength |
> |---|---|---|
> | **AoU-OBSERVABLE** | if the integrator violates it, **something on the wire changes** and they can detect it | a real, if partial, countermeasure: the machine participates |
> | **AoU-SILENT** | if the integrator violates it, **nothing anywhere changes**; the only trace is that they did not read a document | **not a countermeasure. A transfer, outstanding.** |
>
> ⚑ **AoU-S3 SAYS THE QUIET PART ON ITS OWN FACE AND THIS SEAT SCORED THE ROW ANYWAY:**
> *"The component does not currently tell you this on the wire; it tells you here."*
> **That sentence is the definition of AoU-SILENT, written by the author of the row that relied on
> it.** It was published as reassurance about a covered condition. It is a confession.
>
> #### What follows, and it is buildable rather than rhetorical
>
> **An AoU-SILENT can be converted to an AoU-OBSERVABLE by publishing the thing the assumption is
> about.** That is a code change, it is small, and it is the CODE-PRIMARY form this seat's charter
> requires — *the document is the byproduct.*
>
> **Measured this turn: the component publishes 14 keys** (`grep -c 'add("' src/…` → 14). **Not one
> of them carries the silence budget actually in force.** A consumer can see `report_period_s` and
> `valid_for_s` and cannot distinguish *"the detector is on and generous"* from *"the detector is
> off and a fallback is running"* — the two configurations AoU-S3 exists to separate.
>
> > **⛑ FSE-C1 — THE ONE-KEY COUNTERMEASURE. Publish `silence_budget_s`: the EFFECTIVE budget,
> > the same `silence_budget` the code already computes at `cpp:1133-1136`.** With it, AoU-S3
> > stops being a request that someone read a document and becomes a fact on the wire that a
> > three-line consumer can act on. **`src/` is CPP's this turn — specified, not written.**
>
> **Until such a key exists, every AoU below marked SILENT is recorded as what it is: an
> unmitigated residual this component has offered to transfer and nobody has accepted.**


These extend the assumptions-of-use block at `hpp:623-765` (AoU-1 at `:629`, AoU-5 at `:724`) and do not replace them. **An integrator who cannot
accept one of these must not rely on `zone_decision` for that property.**

> ⛑ *CLASS: AoU-OBSERVABLE — a violation shows up as a robot that did not stop, which you will see. The gating code is yours and its absence is visible to you.*
>
> **AoU-S1 — YOU MUST TREAT THIS COMPONENT'S OUTPUT AS AN OFFER, NEVER AS AN INTERLOCK.**
> It reports; it cannot stop anything. `CostmapFilter::updateCosts()` sets `current_ = true`
> unconditionally after `process()` returns and is declared `final`, so this component cannot
> drive nav2's own *"do not plan on this yet"* channel. **The gating code is yours.**

> ⛑ *CLASS: AoU-OBSERVABLE — the value you must not proceed on is on the wire, and a fifth value would be too.*
>
> **AoU-S2 — PROCEED ONLY ON `enforced: yes`. A WHITELIST, NEVER A BLACKLIST.**
> Four values ship today: `yes`, `pending`, `unknown`, `NO`. A blacklist fails open the moment a
> fifth is added — and a fifth was added on 2026-09-06. Treat every value other than `yes` as
> *the zone's limits are not known to be applied.*

> **AoU-S3 — ⚑ IF YOU DISABLE A DETECTOR, YOU MUST NOT READ THE VALUE IT GATED.**
> Setting `costmap_silence_timeout <= 0` does not merely suppress a warning: it makes
> `watching: yes` and `enforced: yes` unfalsifiable (**PI-1**). If you set it, you must supply
> your own liveness check or stop consuming `enforced` altogether. **The component does not
> currently tell you this on the wire; it tells you here.**
>
> ⛑ **CLASS: AoU-SILENT — and AMENDED 2026-09-06 ON TWO COUNTS.**
> **(1) Its stated hazard is GONE.** `costmap_silence_timeout <= 0` no longer makes `enforced`
> unfalsifiable: a fallback budget of `liveness_period x 2.5` applies and `STALE` still fires
> (**PI-1, closed, executed this turn**). **You may set it.** What you lose is the *explicit*
> report and the ability to choose the budget — not the detector.
> **(2) The sentence above is this document's own proof that a documented assumption is not a
> countermeasure** (§5). It is quoted in that ruling against the row it was written to support.

> ⛑ *CLASS: AoU-SILENT — nothing distinguishes a correct reading of this field from the converse reading, which is the one a consumer will infer.*
>
> **AoU-S4 — ⚑ `valid_for_s` IS NOT A PROMISE AND MUST NOT BE READ AS ONE.**
> It is a **declared expiry for the REPORT**, derived solely from `liveness_period`. It says
> nothing about how long the enforcement CLAIM remains true — per AoU-4 that is unbounded and
> unverified by construction (**PI-3**). It has **no ceiling** (**PI-2**). And at shipped defaults
> it **exceeds** this component's own staleness threshold by 0.5 s. Use it as a lower bound on
> *"this publisher is probably still alive"*, never as an upper bound on *"this zone is probably
> still enforced."*
>
> ⚑ **This document's own package has already ruled on the word.** `CHANGELOG.md:85-86`:
> *"An advisory component may not make a promise about timing."* **That correction reached the
> README and the CHANGELOG and did not reach the header.** `hpp:571` still reads *"a PROMISE with
> a number in it"* and `hpp:757` *"a PROMISE WITH A NUMBER"* — in the file an integrator compiles
> against. **This is the second occurrence of that exact propagation shape in this file within one
> day**; the first is recorded at `hpp:689`, where the whitelist correction *"reached the
> README and AoU-2 below and MISSED THIS LINE."* Left named here rather than repaired silently:
> FSE does not hold the pen on `src/` or `include/`.

> **AoU-S5 — SET YOUR OWN LIVENESS FLOOR, AND DO NOT EXPECT `DEADLINE` QoS TO WORK.**
> `costmap_silence_timeout` governs the verdict; `liveness_period` governs how fast you learn it
> (**PI-5**). Keep `liveness_period` well below `costmap_silence_timeout` or the budget you set is
> not the budget you get. ⚑ **And the `DEADLINE` QoS the README recommends will not match this
> publisher (PI-4)** — use a `diagnostic_aggregator` staleness rule, or your own timer over
> `report_seq`, which is the one field a stopped clock cannot fake.

> ⛑ *CLASS: AoU-OBSERVABLE as of the `describeUnansweredTarget()` change — and AoU-SILENT until then. The failure
> sentence now tells you which of the two causes it observed and prints your own YAML spelling
> beside the resolved name, so a violation of this assumption arrives at the operator instead of
> waiting in a document. ⚑ **This is the one assumption in this section that has been converted,
> and it is the worked example of what §5 asks for.***
>
> **AoU-S7 — ⚑ A RELATIVE `node:` RESOLVES AT THE COSTMAP'S *PARENT* NAMESPACE, NOT ITS OWN.**
> ⛑ **AND IF YOUR TARGET LIVES INSIDE THE COSTMAP'S OWN NAMESPACE, USE AN ABSOLUTE NAME — that
> case worked before the join was introduced and does not now (PI-16).** The arithmetic, from nav2's source at
> `1.5.1`: `parent = ns.substr(0, ns.rfind("/"))`, so a costmap node in `/local_costmap` has parent
> `""` and your relative `foo` becomes `/foo`, not `/local_costmap/foo`. **An absolute `/name` is
> unaffected in every topology, which is why it is the only spelling this document recommends.**
> `src/zone_parameter_filter.cpp:387` applies nav2's `joinWithParentNamespace()` to your `node:`
> field, and that function takes `node->get_namespace()` and **strips one level**
> (`layer.cpp:88-96`, read in the image). So for a costmap node at `/robot1/local_costmap`, a
> relative `node: controller_server` addresses **`/robot1/controller_server`** — the peer level,
> which is where nav2 puts servers. **Verified as DELIVERING, not merely as non-breaking:** SC-7
> drives a namespaced host with the target at the parent level and reaches `enforced: yes`; SC-8 is
> its control and confirms a target left at **root** is correctly *not* reached. ⚑ **Both are the
> only cases in this package that run outside root namespace** — every other test runs where a
> joined and an unjoined name resolve identically. **An absolute `/name` is unaffected.**

> **AoU-S8 — ⚑ THE SENTENCE IS FOR A PERSON. THE TOKEN IS FOR YOUR CODE. NEVER PARSE THE SENTENCE.**
> `st.message` is prose, it is not versioned, and it has been reworded three times in two days
> (`cpp:1264`, `cpp:1268`, `cpp:1302`). **Gate on `enforced` (AoU-S2); render the sentence to a
> human unchanged.** ⚑ **And read the sentence's limits before you put it on a screen: it says what
> THIS FILTER knows and never what to do with the machine.** It carries an imperative — *"Decide as
> if this zone is unchecked"* (`cpp:1283`, `cpp:1296`) — which is a statement about **this zone's
> limits**, not about your robot's motion. **Your operator will read it beside a moving robot. The
> subject is the zone.** *(CLASS: AoU-SILENT — nothing on the wire stops you parsing it.)*

> **AoU-S9 — ⚑ THE NEVER-DRIVEN WARNING FIRES THROUGH EVERY ORDINARY BRINGUP. BUDGET FOR IT.**
> The heartbeat publishes from configure, before the costmap update thread exists, so
> *"THIS FILTER IS NOT WATCHING. It has not been driven even once"* is **expected** in the first
> moments and is **not** a fault. The component differentiates the two never-watching sentences
> deliberately (`cpp:1285-1292`). **If you page an operator on `STALE` without a hold-down, you
> will page them at every startup, and they will learn to discount the one that matters (PI-15).**
> **Apply a hold-down of at least a few `liveness_period`s.** *(CLASS: AoU-SILENT.)*

> **AoU-S10 — ⚑ IF YOU DISABLE THE HEARTBEAT (`liveness_period <= 0`) YOU GET NO PERIODIC REPORT,
> NO DEADLINE, AND `valid_for_s: 0.0`.** No timer is created (`cpp:251`), so nothing publishes
> except on change; `declaredValidForS()` returns `0.0`, so every message declares itself expired
> on arrival; and **no `DEADLINE` is offered, so do not request one** — you would receive nothing
> (PI-4's failure mode, re-entered by the other door). **Checked this turn: this configuration does
> not re-open PI-1.** It is a worse configuration and an honest one. *(CLASS: AoU-OBSERVABLE —
> `valid_for_s: 0.0` and the absent heartbeat are both visible to you.)*

> **AoU-S11 — ⚑ THE STARTUP LOG IS NOT PART OF THE CONTRACT, AND ONE LINE OF IT IS WRONG TODAY.**
> With `costmap_silence_timeout: 0` the `INFO` at `cpp:255-258` reports the stopped-costmap
> threshold as **`0.000s`**; the budget actually in force is `liveness_period x 2.5`, which the
> `WARN` three lines above states correctly (**PI-10**). **Take your budget from `valid_for_s` and
> `report_period_s` on the wire, never from the log.** *(CLASS: AoU-SILENT until PI-10 is fixed.)*

> ⛑ *CLASS: AoU-OBSERVABLE — but only if you requested the DEADLINE (PI-4), and only if `liveness_period > 0` (AoU-S10). Otherwise SILENT.*
>
> **AoU-S6 — SILENCE ON `zone_decision` IS *"I AM NOT WATCHING"*, AND THE NODE'S DEATH IS OUTSIDE
> THIS COMPONENT.** If the node dies the timer dies with it and the last message stands. Nothing
> running inside a process can announce that process's own death. **The complete answer lives in
> your process and is not built here.**

---

## 6 · ⚑ What is UNVERIFIED, and stays UNVERIFIED rather than *cleared*

> ### ⛑ SECOND SELF-CORRECTION, 2026-09-06 — THE INSTRUMENT DID NOT COVER ITS OWN SUBJECT.
>
> **This section already carried one correction against this document's author. Here is the
> second, and it is larger, because the first was a false sentence and this is a blind
> instrument.**
>
> **IT DID NOT SAY ANYTHING. That is the defect.** `"Decide as if"` appears in this document
> **zero** times. So does `st.message`. **The analysis whose entire purpose is to ask what a
> misleading report does to a person never once took the sentence the person reads as its unit of
> analysis.** Every one of the nine original rows is about a key/value pair.
>
> **WHY IT HAPPENED, and the answer is not carelessness.** The unit of analysis was inherited
> from the failure history: four generations of the `enforced` token, so the token became the
> object. **The instrument was shaped by the defects it already knew about, which is exactly the
> shape that cannot find a new one.** ⚑ **And this document's own §3 names that class:** the
> reassuring value is computed from the absence of negative evidence, and *"absence of negative
> evidence is produced both by 'all is well' and by 'nothing is looking.'"* **Zero findings on the
> sentence channel was produced by nothing looking**, and this section published it as coverage.
>
> **THE FALSIFICATION, so this is a measurement and not a confession.** If the sentence channel
> were defect-free, a blind instrument would cost nothing. **It is not: three defects have been
> found and fixed there, all marked `⛑` in the source, none by this analysis (§4A, s-1..s-3) —
> and a fourth is open and confirmed by execution as of this turn (PI-10).** Four known defects,
> zero found here.
>
> **WHAT IS NOW TRUE.** §4A exists, carries six rows, and specifies three oracles. **WHAT IS STILL
> NOT TRUE:** none of the three oracles is written, so §4A is analysis without an executable
> invariant behind it — **which is precisely what this seat's own charter cuts as papers-as-end.**
> `src/` and `test/` are held by another seat this turn; that is the reason and it is not a
> discharge.
>
> ⚑ **AND THIS SEAT MAY NOT CLOSE THIS.** It named the gap against itself, which is the one thing
> it is competent to do here; **whether §4A is the right scope, and what else this document owes,
> is DIA's to rule (OPS-064(C), bypass: none).** A producer that certifies its own self-correction
> has produced a second, better-worded blind spot. **Recorded OPEN, pending DIA.**

> ### ⛑ AND A THIRD, WHICH IS NOT THIS DOCUMENT'S FAULT AND IS STILL THIS DOCUMENT'S PROBLEM.
>
> **Every line-number citation in this document was re-derived twice during the turn that wrote
> this sentence, because the file being cited was rewritten underneath it — measured, not
> inferred: `src/zone_parameter_filter.cpp` was written at 18:42:44 and
> `include/hubot/zone_parameter_filter.hpp` at 18:41:58, mid-turn, growing by 87 and 43 lines.**
> The first repair pass was stale before it was saved.
>
> **The closure plan sequencing this work states that the two lanes are *"disjoint, so they run
> together"* — disjoint because they touch different FILES. ⚑ THAT DISJOINTNESS IS FALSE, and it
> is false in exactly the direction that manufactures stale citations: this document's CONTENT is
> a function of the other file's LINE NUMBERS.** Two seats editing disjoint files are not working
> on disjoint state when one cites the other by position.
>
> **The primitive that is missing is named, and it is not "more care":** *there is no derived
> citation.* Every position in this corpus's prose is a literal typed by hand, and **nothing
> re-derives it, so it is correct only until the next save by anyone.**
>
> **THE COUNTERMEASURE, and it inverts the direction the loom currently reaches for.** A gate that
> CHECKS numbers after the fact is a treadmill that runs one edit behind. A citation written as an
> **anchor** cannot go stale at all, because it contains no number:
>
> - **Cheap and available today** — for citations already of the form `` `file:N` (`Anchor`) ``,
>   where the parenthetical is a literal token in the source, the number is **mechanically
>   derivable**: find the anchor, compare. **Every citation in this document was re-derived by
>   exactly that method this turn, by grep, in one pass.** It is satisfiable, its negative control
>   is trivial (perturb a number → RED), and **it is the subclass `hpp:495-637 (AoU-1..AoU-5)`
>   belonged to.**
> - **The real fix** — let the prose cite `` `cpp@notWatching()` `` and have the loom render or
>   verify the position. Then a concurrent edit **cannot** falsify a document, rather than being
>   caught after it has.
>
> ⚑ **Which is why this document now writes every load-bearing citation as `number` + `anchor`.**
> A reader whose tree has moved can `grep` the anchor and re-find the line themselves. **The
> citations below are true against the tree as it stood at 18:42:44 on 2026-09-06 and will go
> false at the next save. The anchors will not.**


⚑ **CORRECTED 2026-09-06, against this document's own author, and the false sentence is kept
because it is the reason the correction exists.**

**IT SAID:** *"Measured this turn: `/opt/ros` does not exist on this machine, `colcon` is not
installed, and no released `nav2_costmap_2d` carries the `ZONE_PARAMETER_FILTER` constant this
package's source names. **So the whole component cannot be built here**, and every finding above is
from reading the source, not from running it."*

**The last clause was true. The one before it was false.** Docker is present; the image
`nav2-lyrical-main-compat:latest` (8.91 GB) is on disk and carries `ZONE_PARAMETER_FILTER = 4` at
its installed `filter_values.hpp:51`; and a built `hubot` workspace with a shared library and six
test executables was sitting in this session's own scratchpad, dated the same morning. **The
instrument was `ls /opt/ros` and `which colcon` on the HOST, and it stopped there** — so it could
not have surfaced the counter-example, which is this corpus's own test of whether a measurement
measured anything (**Vision 77** — *go to the actual place and see the actual thing; second-hand
descriptions lie*).

⚑ **FBR had corrected this exact error, in another seat's file, the day before**
(`outputs/fbr/hubot_clean_build_reproduction_2026_09_05.md` §1: *"'no ROS installation exists on
this host.' True of the **host**, and misleading about the reproduction environment."*). It was
reproduced here one day later.

⚑ **AND IT IS THIS DOCUMENT'S OWN SUBJECT, COMMITTED IN THIS DOCUMENT.** PI-CLASS says the
reassuring value is computed from the absence of negative evidence, where absence is produced both
by *"all is well"* and by *"nothing is looking."* This section did the mirror image: it presented an
absence of evidence as **structurally unavailable** when it was merely **unfetched**. The
difference is not pedantic — an `UNVERIFIED` whose stated reason is *impossibility* tells a reader
**nobody could have checked this**, and invites them to accept the residual; an `UNVERIFIED` whose
true reason is *nobody has run it yet* tells them it is **cheaply checkable**, and invites them to
run it themselves. **Those license opposite decisions.**

**WHAT IS TRUE. The evidence class is unchanged — the findings above were derived by reading, and
`UNVERIFIED` is the correct label for them. What was false was the WARRANT.** The execution status
of each is recorded in the table below and is being closed, not excused.

### ⚑ EXECUTED 2026-09-06 (first pass). The predictions were run, and they held.

*⛑ Re-executed later the same day after the code moved: **SC-1, SC-2 and SC-3 are now GREEN**, and
PI-1, PI-2 and PI-4 are ruled CLOSED in §4. The rows below are kept as the record of the defects
that produced the fixes — a table of confirmed defects is what a closure has to be checked against.*

**Environment:** image `nav2-lyrical-main-compat:latest`, `--network=none`, ROS `lyrical`,
`nav2_costmap_2d` 1.5.1 with `ZONE_PARAMETER_FILTER = 4`. Build **exit 0 in 29.9 s, first attempt,
zero errors.**

| claim | status |
|---|---|
| **PI-1** — a disabled silence detector reports safety | ⛑ **WAS CONFIRMED, NOW CLOSED.** SC-1 was **RED** with `enforced=yes watching=yes` from a costmap stopped 1.5 s. **Re-executed 2026-09-06 after the input fix: SC-1 GREEN, SC-2 GREEN.** The pair is the evidence; see the ruling in §4. |
| **PI-2** — declared expiry exceeds the silence budget | ⛑ **WAS CONFIRMED, NOW CLOSED.** SC-3 was **RED** at `valid_for_s=2.500000` against a 2.0 s budget. **Re-executed 2026-09-06: SC-3 GREEN**, the clamp lives in `declaredValidForS()` and announces itself once at configure. |
| *(historical)* **PI-4 as it was** | The record of the defect, kept because it is why the fix exists: SC-5 **RED**, deadline arm **0 messages**, control arm **22**, and the rmw said it verbatim — *"requesting incompatible QoS. **No messages will be sent to it.** Last incompatible policy: **DEADLINE_QOS_POLICY**"* |
| **PI-4** — the recommended `DEADLINE` QoS cannot match this publisher | ⛑ **WAS CONFIRMED, NOW CLOSED** by an offered deadline at `cpp:228-232`. ⚑ **Not re-executed by this seat this turn** — the two-arm QoS harness was not re-run, so this closure rests on the source read plus the package's own SC-5. **The weakest of the three closures, and labelled as such rather than levelled up.** |
| **PI-6** — `unknown` on the never-driven branch | **REFUTED as a value defect.** SC-4 **GREEN** — the branch already reports `unknown` + `STALE` correctly. It was **unasserted**, not wrong. It is asserted now. |
| **AoU-S1** — the filter cannot carry its fault on `Layer::isCurrent()` | **HOLDS TODAY.** SC-6 **GREEN**, armed as a tripwire: it goes RED the day the upstream `setCurrent` reordering lands, which is the signal to rewrite AoU-S1 rather than to fix the test |
| PI-3, PI-5, PI-7 and every line/quote citation | **CONFIRMED by source read** — each carries a file position checkable without building |
| **Regression check on the closed set** | **GREEN.** Full suite re-run — **33 gtest cases, 2 failures, both of them SC-1 and SC-3, i.e. only the new oracle.** No pre-existing case regressed. |

⚑ **PI-4 was the one finding derived from a specification rather than from source read on disk, and
it was published as `CONFIRMED-BY-SPEC / UNVERIFIED-BY-EXECUTION`. It is now the best-evidenced
entry in this table.** ⚑ **And execution found something the reasoning had not:** the
incompatibility warning is emitted **on the publisher's node**, not the subscriber's. An integrator
who follows `README.md:557` gets **silence in their own process**, while the diagnostic explaining
why appears in a log that is **ours**. The failure is silent exactly where it needs to be heard.

### ⛑ RE-EXECUTED 2026-09-06 — INCLUDING CODE ITS AUTHOR COULD NOT COMPILE

**The commit that added `describeUnansweredTarget()` states in its own message: *"colcon cannot run on this host: nothing here was
compiled or executed by its author."* It added 95 lines to `src/`, 43 to the header and a
447-line new test suite.** Saying so was the right act and it left a real gap: **a safety-relevant
change that has never been compiled is not a change whose behaviour anyone knows.** This seat
holds no pen on `src/`, but it can run a compiler, and the publish is permanent.

**Run this turn — image `nav2-lyrical-main-compat:latest`, `--network=none`, ROS `lyrical`,
`rmw_fastrtps_cpp`, clean workspace, `-DCMAKE_BUILD_TYPE=Release`:**

| | |
|---|---|
| **build** | ⛑ **exit 0 in 35.9 s**, first attempt |
| **suite** | ⛑ **52 tests, 0 errors, 0 failures, 0 skipped** — 42 gtest cases across 8 suites plus 10 CTest targets |
| **`namespaced_target_resolution_test`** | ⛑ **4 of 4 GREEN** — the suite recorded as needing a robot, which needed one line |
| **`sotif_gate_inertness_test`** | ⛑ **8 of 8 GREEN** — SC-1 … SC-8, including the SC-1/SC-2 pair that closes PI-1 |

⚑ **WHAT THIS DOES AND DOES NOT ESTABLISH, because a green suite is exactly the observable this
document exists to distrust.** It establishes that the tree **compiles** and that **every oracle
the package currently has passes**. It does **not** establish that the new sentences are correct
for a person — **no oracle in this package reads a sentence for MEANING**, only for forbidden
tokens (SC-9/SC-11). **PI-10 through PI-15 remain open with 0 of 3 of their specified oracles
written, and PI-10 is red on a surface nothing tests.** A suite that is green on everything it
measures says nothing about what it does not measure, which is §3 of this document applied to
this document's own evidence.

### What remains unexecuted, and stays UNVERIFIED rather than *cleared*

| item | status |
|---|---|
| **The rmw generalisation for PI-4** | ⚑ **UNVERIFIED.** The result above is true of the rmw in this image. DEADLINE Request/Offered handling is implementation-specific; **a single-rmw answer does not generalise to your stack.** Verify on yours. |
| **The real-stack claim under the whole liveness argument** | ⚑ **EXERCISED — AND FOR A DAY THIS CELL DID NOT SAY ON WHAT.** What runs: real `Costmap2DROS` with its own `map_update_thread_`, nav2's own `controller_server`, target node **out of process**, genuine BT `ClearEntireCostmap` through `libnav2_clear_costmap_service_bt_node.so`. **Yielded PI-8 and PI-9.** ⚑ **The 2026-09-06 run happened on a LOCAL, UNPUBLISHED image in which every nav2 package was built FROM SOURCE** (`nav2_costmap_2d` 1.5.0 in one overlay, 1.5.1 in another, **no released nav2 deb installed at all**), and the harness it cites **was not in this repository** — so a reader following the citation reached nothing, and could not have reproduced it if she had. ⚑ **RE-EXERCISED 2026-09-07 ON RELEASED nav2, AND THE FIRST ATTEMPT FAILED.** On ubuntu 26.04 resolute with nav2 1.5.1 from `packages.ros.org`, the costmap and the filter came up (`Initialized costmap filter "zone_parameter_filter"`) and `controller_server` then died at configure: *Failed to create controller … `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` … does not exist* — the harness's config named that controller and its manifest never declared the package, which the source-built image had supplied by accident. **`configure rc=0, activate rc=1` on all three conditions.** One `<exec_depend>` closed it; the same three conditions then returned **`configure rc=0 / activate rc=0 / bt_recovery rc=0`**, the positive control logged a real cross-process `set_parameters` round trip at the target, and `map_update_thread_` was measured from the costmap node's own DEBUG log at **5.000 Hz (n=92 over 18.200 s)**. **The harness is now in this repository at `hubot_live_stack/` and CI runs those three conditions on released nav2 on every push** (`gate.yml`, job `live-stack`). ⚑ **Still UNVERIFIED within it:** no sensor layers, so filter-vs-obstacle-layer contention is unexercised; no goal sent, so the costmap thread is real but the LOAD is not; simulated TF; synthetic mask; single host, so cross-machine DDS is untested. Not a robot, not hardware. ⚑ *Substrate and citation repaired by BDE 2026-09-07 from measurement; the row's safety DISPOSITION is FSE's and is untouched.* |
| PI-3, PI-5, PI-7, PI-C2, PI-C5 oracles | **not yet written** — each a single case in the existing fixture |

---

## 7 · ⚑ THE PROMISE LEDGER — every vision this analysis rests on, and what keeps it

**Komada-voice, 2026-09-06: *"Make them learn from Sakichi 100 Visions with each promises."***

**A vision is a promise, not a slogan, and a promise with nothing that can break it is decoration.**
So each one below carries three things: its **verbatim text**, the **promise** it makes of this
component, and **the oracle that fails if the promise is broken** — or the honest statement that
nothing keeps it yet.

⚑ **Why the text is inlined rather than cited by number.** This document ships to integrators
*outside* the repository that holds the corpus and its resolver. **A citation the reader cannot
resolve is worse than none — it looks resolved.** Every quote below was verified this turn with
`scripts/vision.sh --verify <n> "<quote>"`, the instrument that refuses a fabricated attribution;
all five returned `VERIFIED`, and a deliberate false attribution was run as a negative control and
was **refused**. The promises travel with the findings.

> ### ⚑ AND THE FIRST THING THE LEDGER FOUND WAS AGAINST ITSELF.
>
> **Vision 14** — *"Silent failure is the anti-Jidoka — a function that returns a success-shaped
> value while the operation failed is a loom weaving through a broken warp."*
>
> **That is PI-CLASS. It is §3 of this document, in one sentence, and the corpus had it all along.**
> This package had already cited it — `src/zone_parameter_filter.cpp:585`, written the day before
> this analysis — and the analysis **derived the class independently from four generations of a bug
> without ever resolving it.** Both routes reach the same place; only one of them is cheap.
>
> ⚑ **That is the whole lesson of *"learn from the visions."* A seat that goes to the genba and a
> seat that reads the corpus both arrive — but the seat that reads first arrives on turn one and
> spends the rest of its budget on what the corpus does NOT already know.** §3 is renamed in
> substance, not in text: **PI-CLASS is Vision 14 applied to a reporting component.**
>
> *(Attribution verified this turn with `--verify`; a false attribution to the same vision was run
> as a negative control and refused. The citation at `cpp:479` was a bare number with no text (repaired 2026-09-06 to `cpp:612`, the line that names the vision) — it
> survives only because the surrounding sentence happens to paraphrase it. **FSE does not hold the
> pen on `src/`; named for CPP, not edited.**)*

| vision (verbatim) | the promise it makes of hubot | what keeps it |
|---|---|---|
| **14** — *"Silent failure is the anti-Jidoka — a function that returns a success-shaped value while the operation failed is a loom weaving through a broken warp."* | ⚑ **The root promise. `enforced` must never be success-shaped when the operation did not succeed — or was never attempted.** | **The entire table.** SC-1, SC-3, SC-5 RED · SC-2, SC-4, SC-6 GREEN · six closed generations · `CPP_N3`, `FSE_F`, `MF-1`, `MF-6`. ⚑ **This is the promise the package exists to keep, and it is the one it has broken five times.** |
| **9** — *"The operator must not be the last line of defense against defects — the machine itself must catch them."* | An integrator must not have to **notice** that a zone went unenforced. The component must say so. | ⛑ **SC-1 GREEN as of this turn — the promise is now KEPT on the token channel.** ⚑ **AND BROKEN ON THE OTHER TWO, which is the finding of this revision.** (a) **The sentence channel has no oracle at all** (§4A: four known defects there, three fixed by other hands, one open) — the field a human reads is the field the machine does not check. (b) **§5 rules that twelve assumptions of use are not countermeasures**: every AoU-SILENT makes the *reader* the last line of defence, which is this vision's exact prohibition, committed twelve times in one section. |
| **88** — *"SRE pages on SLO burn — the production system self-reports when it has departed from contract, so humans investigate rather than stand watch."* | ⚑ **This is PI-CLASS stated positively.** hubot exists to self-report departure from contract. The defect family is that it self-reports **conformance** when nothing is looking. | **SC-1, SC-3, SC-5** (all RED) · **SC-2, SC-4** (GREEN — it *does* report correctly when the mechanism is live). The whole table is one question: does this system self-report departure, or only self-report? |
| **20** — *"Stopping must be cheap, or operators will hesitate; design the halt to cost less than the defect."* | A failed parameter set must **degrade and keep navigating**, never kill the node. | **Kept, and it is the package's reason for existing** — **named in its own source, in words rather than by ordinal**, at `cpp:938` and `cpp:988` (*"Sakichi's rule: the halt must cost less than the defect"*). Oracles: `test_degrade_not_abort`, `HUBOT_A/B`, `survival_harness_test`, all **GREEN**. |
| **77** — *"Genchi genbutsu — go to the actual place and see the actual thing; second-hand descriptions lie."* | Findings about this component must come from the component, not from reading about it. | ⚑ **BROKEN BY THIS DOCUMENT'S OWN AUTHOR AND REPAIRED IN §6** — "the package cannot be built here" was written from `ls /opt/ros` on the host while docker, the image and a built workspace were all present. **Now kept**: every finding above was executed. ⚑ **AND NOW KEPT ON THE SECOND LIMB TOO**: a build seat stood up a real `Costmap2DROS` with its own `map_update_thread_` at a measured 5.000 Hz, nav2's own `controller_server` binary, the target node in a **separate OS process**, and a genuine BT `ClearEntireCostmap`. ⚑ **Going there immediately produced PI-8 and PI-9 — two findings the fixture was STRUCTURALLY incapable of surfacing.** The promise paid the moment it was kept. |
| **45** — *"Invention that does not reach the market is not yet complete — commerce is how service lands on the user."* | Whatever this analysis establishes, it establishes about something **nobody can install**. | ⚑ **NOTHING KEEPS THIS, AND NOTHING HERE CAN.** The completing act is an upstream release, not ours. ⛑ **AND IT NOW HAS TEETH IT DID NOT HAVE**: §5 rules an assumption of use is a TRANSFER that only takes effect on ACCEPTANCE, and **with no integrator there is no acceptor** — so this row is not a caveat on the analysis, it is the reason twelve assumptions of use currently mitigate nothing. **The two facts are one fact.** |

> ### ⛑ ADDED 2026-09-06 — VISION 14 IS THE ONE THIS REVISION MOVED, AND IT MOVED SIDEWAYS.
>
> **Vision 14** — *"Silent failure is the anti-Jidoka — a function that returns a success-shaped
> value while the operation failed is a loom weaving through a broken warp."*
>
> **Three of the four generations this table tracked are CLOSED as of this turn, and two of those
> closures were verified by running the code rather than reading it.** That is real and it is the
> good news of the revision.
>
> ⚑ **And the promise moved to a channel this document was not watching.** A sentence that says
> *"the costmap is reported as stopped after 0.000s"* when the budget is 0.25 s is a
> **success-shaped value about the component's own configuration** — Vision 14, in the operator's
> log, at every bringup, in the one configuration the whole PI-1 family is about. **Found by
> running the code for eight seconds; not found by the analysis that was looking for exactly this
> and had been for two days.** The class was never about the `enforced` token. It was about
> **anything this component says that is shaped like reassurance**, and the token is only where we
> happened to be looking.

**How to read this ledger, and it is the point of it.** ⚑ **Three of the five promises were kept
only after something failed** — Vision 9 caught two unguarded entries, Vision 77 caught a false
sentence in this document, Vision 88 named the class the whole table is about. **A promise that has
never cost its holder anything has not been tested.** Vision 20's row is the one to be least
comfortable about: it is marked kept, its oracles are green, and **nothing on this page challenged
it this session.**

---

## 8 · Scope, provenance and audit status

⛑ **HOW TO CHECK THIS DOCUMENT'S CITATIONS WITHOUT TRUSTING IT.** Every load-bearing citation is
written `` `file:N` (`anchor`) ``. Run `grep -n '<anchor>' <file>` and compare. **That is not a
courtesy — it is the check this document argues the package's own loom should automate** (§6), and
it is stated here so a reader can run it on the day they read this rather than the day it was
written.

Produced by **FSE** under a QM/advisory ceiling. **FSE is a producer and is not its own auditor**
— this document is unaudited and routes to **AAA + DIA**. It **attaches to** existing records and
re-authors none; measured this turn, **no `SAFETY_BOUNDARY.md` exists for `hubot`**, so nothing
was displaced.

Governing thesis: `D-VGC264-1` — *hubot is a bridge between robot output and human sensor.*
The insufficiency class above is a **robot→human** defect: the rate-reduction direction, where the
compression is required to preserve the decision and instead preserved a reassuring word. It says
nothing about the human→robot direction, which `D-VGC264-1` §3 measured as absent and named open.
