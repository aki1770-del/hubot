# hubot — SOTIF performance-insufficiency analysis and assumptions of use

**Component:** `hubot::ZoneParameterFilter` · **Standard frame:** ISO 21448 (SOTIF)
**Class:** QM / advisory / information-only · **Authored:** 2026-09-06 by FSE
**Audited by:** AAA + DIA — *this document is a producer's output and has not been audited yet.*

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
> **tell a person, in words, whether those limits actually took** — before the risk arrives, as an
> offer, with the person keeping the decision.

Two properties of that sentence do the work in everything below:

1. **The report's subject is a claim about another process's state** — a target node's live
   parameter value — which this component observes only indirectly, only on request, and only
   while something is driving it.
2. **The report is consumed as permission.** The documented consumer rule is a whitelist:
   *proceed only on `enforced: yes`*. So the reassuring value is the one that moves a robot.

---

## 3 · ⚑ THE CLASS — one insufficiency, five generations and counting

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
| **⚑ 5** | **the liveness gate is configured inert** | **`costmap_silent_` can never become true** | **OPEN — see PI-1** |

**The package's own diagnosis of generations 1–4 is correct and is quoted here because it is the
reason generation 5 exists** (`include/hubot/zone_parameter_filter.hpp:319-321`):

> *"Every fix went into a caller and every new caller arrived without it. So this one goes into
> the VALUE."*

⚑ **The gate moved into the value. Its INPUTS did not.** `notWatching()` reads `costmap_silent_`,
which is computed at `src/zone_parameter_filter.cpp:923-924` as
`(costmap_silence_timeout_ > 0.0) && (age > costmap_silence_timeout_)`. **A gate cannot report its
own inertness, and this one does not.** Generations 6 and 7 will come from the same place unless
the invariant, not the instance, is held.

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
| **PI-1** | **The silence detector can be configured inert, and the component then makes an affirmative liveness claim instead of withholding one.** | `costmap_silence_timeout <= 0` — **documented** at `README.md:440` as *"keep the heartbeat but never report a stopped costmap"* — **and** the costmap subsequently stops. | `watching: yes`, `enforced: yes`, level `OK`, *"Zone N is in force."*, republished every `liveness_period`, indefinitely. | The integrator asked **not to be told** about a stopped costmap. They were given a **positive claim that it is running.** This is generation 3 restored by one line of YAML, past a value-level gate built to prevent it. | `cpp:923-924` (short-circuit) · `hpp:333` (`notWatching()`) · `cpp:1080` (`watching` derives from it) · **`grep -rn "silence_timeout" test/` → only `2.0` and `0.3`; the `<= 0` path has ZERO tests** | ⚑ **OPEN — CONFIRMED BY EXECUTION 2026-09-06 (SC-1 RED)** |
| **PI-2** | **The declared expiry exceeds the publisher's own threshold for calling itself stale, and neither number has a ceiling.** | Shipped defaults. No misconfiguration required. | `valid_for_s: 2.5` while `costmap_silence_timeout` is `2.0`. | `README.md:248` instructs: *"If `now - header.stamp > valid_for_s`, stop believing the message."* Read literally — and it is written to be read literally, *"by three lines of consumer code"* — the consumer keeps believing an `enforced: yes` for **2.5 s** after the last message, while the publisher would have disowned its own reading at **2.0 s**. Widens without bound: `liveness_period: 30.0` → `valid_for_s: 75.0`, and on a moving platform that is a distance, not a time. | `cpp:59` (`kValidForPeriods = 2.5`) · `cpp:1086` (product) · `hpp:467` (`2.0` default) · `README.md:248` · **no clamp on either parameter anywhere in the translation unit** | ⚑ **OPEN — CONFIRMED BY EXECUTION 2026-09-06 (SC-3 RED)** |
| **PI-3** | **`valid_for_s` bounds the freshness of the REPORT; it does not bound the validity of the CLAIM, and the two are presented as one number.** | Any third party sets the same parameter on a target after this component's confirmation. | A message inside its `valid_for_s` window carrying `enforced: yes` for a value that is no longer set. | The component's own AoU-4 already states that a confirmed set means *acceptance*, not *still holding* — **unbounded in time, unverified by construction.** So the claim can go false at any instant, with no bound at all, while the only number a consumer is given to reason about expiry is a **publisher-liveness** number. A consumer told *"believe it for `valid_for_s`"* will infer the converse. The converse is not warranted. | `hpp:590-594` (AoU-4) · `cpp:1074-1079` (the field's own rationale, which is about node death only) | **OPEN — inherent; disclosure is the countermeasure** |
| **PI-4** | ⚑ **The only complete countermeasure the component recommends is structurally incompatible with the component's own publisher.** | An integrator follows `README.md:451` and requests a finite `DEADLINE` QoS on their `zone_decision` subscription. | Under DDS Request/Offered matching, an **offered** deadline of infinity does not satisfy any **finite** requested deadline. The subscription does not match; the consumer receives **nothing at all** — which they are most likely to read as a broken topic, not as *"not watching."* | `decision_pub_` is created with a bare `rclcpp::QoS(10)` (`cpp:147`), leaving the offered deadline at the rmw default (infinite). The package tells the integrator to build the one mechanism it prevents them from using. **One-line fix on the publisher side:** offer a deadline derived from `liveness_period`. | `cpp:147` · `README.md:450-454` · `hpp:633-636` | ⚑ **OPEN — CONFIRMED BY EXECUTION 2026-09-06 (SC-5 RED; rmw: "No messages will be sent to it… DEADLINE_QOS_POLICY")** |
| **PI-5** | **Detection latency is bounded by `liveness_period`, not by the `costmap_silence_timeout` an integrator sets to express their budget.** | `liveness_period > costmap_silence_timeout` — e.g. `0.5` / `5.0`. | Silence declared after up to `costmap_silence_timeout + liveness_period`, not `costmap_silence_timeout`. | The parameter named for the budget does not govern the budget. No consistency check exists between the two; the startup warnings at `cpp:113-128` fire **only on `<= 0`**, so a self-defeating pair is accepted in silence. | `cpp:922-925` (age evaluated once per tick) · `cpp:113-128` (warning coverage) | **OPEN — LIVE** |
| **PI-6** | **The `unknown` vocabulary is asserted positively on one of the two branches that produce it.** | A future change alters the never-driven branch's token. | `MF-1` asserts three negatives (`!= yes`, `watching != yes`, `level != OK`); a change emitting `NO` there passes. | The `unknown` / `NO` distinction is load-bearing and the component argues it at `cpp:1003-1010` — STALE not ERROR, because *"nothing was measured and found bad; nothing was measured."* `NO` sends an operator to look for a fault that does not exist. A vocabulary argued this carefully needs a positive oracle on every branch. | `test/maintainer_findings_test.cpp:410-431` (MF-1, negatives) vs `:640-666` (MF-6, positive, stopped branch only) | **CLOSED as a value defect — SC-4 GREEN; the branch was unasserted, not wrong. Oracle now exists.** |
| **PI-7** | **State 0 reports nominal defaults *in force* having issued nothing, when `nominal_defaults` is empty.** | `nominal_defaults` declared empty; robot leaves the mask. | level `OK`, *"Outside any zone; nominal defaults are in force."* — zero sets issued, zero confirmed. | A positive claim about a restoration that did not occur. **Partially mitigated:** a per-parameter config-load warning fires when a state overrides a parameter with no nominal entry (`cpp:352-368`). It does not fire for the wholly-empty case. | `cpp:683-688` (`resetToNominal` iterates an empty map) · `cpp:1036-1038` (the claim) | **OPEN — low reach; disclosure sufficient** |
| **PI-8** | ⚑ **The report reaches the right VERDICT by the wrong REASON when a target is named relatively, and sends an operator to investigate a healthy node.** | `node:` in the YAML has no leading `/`, and the costmap node is namespaced — **which nav2 always does in production** (`/local_costmap/local_costmap`). | `enforced: NO`, reason *"no answer within Ns of the set being issued; an unanswered request is not a successful one."* | `AsyncParametersClient` resolves the relative name against the **owning** node, so a client is built for `/local_costmap/zpf_target_node` — **a phantom the client itself creates.** The missing-client guard at `cpp:728` therefore does **not** fire; the set goes to a service nobody serves. ⚑ **The target was reachable the whole time** — an external `ros2 param set /zpf_target_node` succeeded. This component exists to tell a person *what to go and look at*, and it names a node that is healthy. Only the pending-record count (7 vs 1) separates *refused* from *never addressed*, and nothing on the wire carries it. | Measured in the live stack: absolute name → 1 pending then `enforced: yes`; relative name → 7 pending, target logs nothing, both ending `NO`. ⚑ **Structurally invisible to every in-process fixture, which hosts the target at root namespace where a relative name resolves correctly BY ACCIDENT.** | ⚑ **OPEN — CONFIRMED BY EXECUTION 2026-09-06, real stack only** |
| **PI-9** | **A filter can be loaded, configured, activated and driven ZERO times inside a stack that reports itself healthy.** | No layer in the costmap declares update bounds. `CostmapFilter::updateBounds()` ignores all four bounds arguments by design. | `LayeredCostmap` computes an inverted window (`Updating area x: [9, 1]`) and the `if (xn >= x0 && yn >= y0)` guard around every `updateCosts()` never fires. Configure/activate return 0, the update loop runs at its configured rate, the footprint publishes. | Every ordinary health signal is green while the filter has never executed. | ⚑ **AND HUBOT CAUGHT IT** — from inside the live stack, unprompted: *"THIS FILTER IS NOT WATCHING. It has not been driven even once."* | ⚑ **NOT A DEFECT IN THIS COMPONENT — evidence FOR it.** See below. |

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
> `notWatching()` is `!ever_processed_ || costmap_silent_` (`hpp:333`), and **the first disjunct is
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
| **PI-C2** | `enforced: yes` at startup | `state_initialized_` gate (`cpp:960-963`) | ⚑ **UNCOVERED.** No test asserts on this gate — the only occurrence of `state_initialized_` in `test/` is a **comment** at `maintainer_findings_test.cpp:533`. MF-1 does not cover it: MF-1 exercises the never-driven path, where `notWatching()` already returns `unknown`, **so deleting the `state_initialized_` check would fail nothing.** |
| PI-C3 | `enforced: yes` during silence | liveness timer + `unknown` (`cpp:903-945`, `:892-894`) | **COVERED** — `costmap_silence_liveness_test.cpp` CASE 1, with a negative control that must reproduce the defect |
| PI-C4 | `enforced: yes` before the first tick | `notWatching()` in the value (`hpp:333`) | **COVERED** — `maintainer_findings_test.cpp:410` (MF-1) + MF-2 negative control |
| **PI-C5** | silent unenforcement when no client exists | `markTargetDegraded` on missing client (`cpp:728-736`) | ⚑ **UNCOVERED.** No test configures a target for which no client was built. The `target_exists=false` arm in `costmap_silence_liveness_test.cpp:239` routes at a node that is not running — **a client IS built and the service never answers**, which is PI-C6, a different door. |
| PI-C6 | a set that never answers reported as success | `set_parameters_timeout` deadline (`cpp:785-796`) | **COVERED** — `degrade_at_production_caller_test.cpp:742` (`FSE_F_ATargetThatNeverAnswers…`) |

⚑ **And the closed set was closed against a codebase that has since moved.** In the build workspace
used for this analysis the newest `colcon` test-result directory is **`test-result_2026-09-05_23-34-16`**,
while the binaries are dated **2026-09-06 08:32** and the source gained an entire new suite
(`maintainer_findings_test.cpp`) plus the `notWatching()` fix at **09:55 the same morning**. **That
workspace's record shows no suite run after those changes.** A closed entry that has not been
re-tested since the code moved is the same shape as a green suite that never runs.

**PI-C2 and PI-C5 are therefore not closed. They are unguarded**, and they belong on the open side
until an oracle exists. Neither is expensive: both are single cases in the existing fixture.

---

## 5 · Assumptions of use — what an integrator must accept

These extend `hpp:495-637` (AoU-1..AoU-5) and do not replace them. **An integrator who cannot
accept one of these must not rely on `zone_decision` for that property.**

> **AoU-S1 — YOU MUST TREAT THIS COMPONENT'S OUTPUT AS AN OFFER, NEVER AS AN INTERLOCK.**
> It reports; it cannot stop anything. `CostmapFilter::updateCosts()` sets `current_ = true`
> unconditionally after `process()` returns and is declared `final`, so this component cannot
> drive nav2's own *"do not plan on this yet"* channel. **The gating code is yours.**

> **AoU-S2 — PROCEED ONLY ON `enforced: yes`. A WHITELIST, NEVER A BLACKLIST.**
> Four values ship today: `yes`, `pending`, `unknown`, `NO`. A blacklist fails open the moment a
> fifth is added — and a fifth was added on 2026-09-06. Treat every value other than `yes` as
> *the zone's limits are not known to be applied.*

> **AoU-S3 — ⚑ IF YOU DISABLE A DETECTOR, YOU MUST NOT READ THE VALUE IT GATED.**
> Setting `costmap_silence_timeout <= 0` does not merely suppress a warning: it makes
> `watching: yes` and `enforced: yes` unfalsifiable (**PI-1**). If you set it, you must supply
> your own liveness check or stop consuming `enforced` altogether. **The component does not
> currently tell you this on the wire; it tells you here.**

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
> README and the CHANGELOG and did not reach the header.** `hpp:443` still reads *"a PROMISE with
> a number in it"* and `hpp:629` *"a PROMISE WITH A NUMBER"* — in the file an integrator compiles
> against. **This is the second occurrence of that exact propagation shape in this file within one
> day**; the first is recorded at `hpp:555-563`, where the whitelist correction *"reached the
> README and AoU-2 below and MISSED THIS LINE."* Left named here rather than repaired silently:
> FSE does not hold the pen on `src/` or `include/`.

> **AoU-S5 — SET YOUR OWN LIVENESS FLOOR, AND DO NOT EXPECT `DEADLINE` QoS TO WORK.**
> `costmap_silence_timeout` governs the verdict; `liveness_period` governs how fast you learn it
> (**PI-5**). Keep `liveness_period` well below `costmap_silence_timeout` or the budget you set is
> not the budget you get. ⚑ **And the `DEADLINE` QoS the README recommends will not match this
> publisher (PI-4)** — use a `diagnostic_aggregator` staleness rule, or your own timer over
> `report_seq`, which is the one field a stopped clock cannot fake.

> **AoU-S7 — ⚑ A RELATIVE `node:` RESOLVES AT THE COSTMAP'S *PARENT* NAMESPACE, NOT ITS OWN.**
> `src/zone_parameter_filter.cpp:292` applies nav2's `joinWithParentNamespace()` to your `node:`
> field, and that function takes `node->get_namespace()` and **strips one level**
> (`layer.cpp:88-96`, read in the image). So for a costmap node at `/robot1/local_costmap`, a
> relative `node: controller_server` addresses **`/robot1/controller_server`** — the peer level,
> which is where nav2 puts servers. **Verified as DELIVERING, not merely as non-breaking:** SC-7
> drives a namespaced host with the target at the parent level and reaches `enforced: yes`; SC-8 is
> its control and confirms a target left at **root** is correctly *not* reached. ⚑ **Both are the
> only cases in this package that run outside root namespace** — every other test runs where a
> joined and an unjoined name resolve identically. **An absolute `/name` is unaffected.**

> **AoU-S6 — SILENCE ON `zone_decision` IS *"I AM NOT WATCHING"*, AND THE NODE'S DEATH IS OUTSIDE
> THIS COMPONENT.** If the node dies the timer dies with it and the last message stands. Nothing
> running inside a process can announce that process's own death. **The complete answer lives in
> your process and is not built here.**

---

## 6 · ⚑ What is UNVERIFIED, and stays UNVERIFIED rather than *cleared*

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

### ⚑ EXECUTED 2026-09-06. The predictions were run, and they held.

**Environment:** image `nav2-lyrical-main-compat:latest`, `--network=none`, ROS `lyrical`,
`nav2_costmap_2d` 1.5.1 with `ZONE_PARAMETER_FILTER = 4`. Build **exit 0 in 29.9 s, first attempt,
zero errors.**

| claim | status |
|---|---|
| **PI-1** — a disabled silence detector reports safety | ⚑ **CONFIRMED BY EXECUTION.** SC-1 **RED**. From a costmap stopped 1.5 s, on the wire: `level=0 (OK) · enforced=yes · watching=yes · costmap_age_s=1.506290 · report_seq=33 · "Zone 3 is in force."` |
| **PI-2** — declared expiry exceeds the silence budget | ⚑ **CONFIRMED BY EXECUTION.** SC-3 **RED** at shipped defaults: `valid_for_s=2.500000` against a 2.0 s budget, on `pending` *and* `yes` messages alike |
| **PI-4** — the recommended `DEADLINE` QoS cannot match this publisher | ⚑ **CONFIRMED BY EXECUTION — and the middleware said it verbatim.** SC-5 **RED**: deadline arm **0 messages**, default-QoS control arm **22**, `incompatible_qos` event **fired**. rmw log: *"New subscription discovered on topic '/zone_decision', requesting incompatible QoS. **No messages will be sent to it.** Last incompatible policy: **DEADLINE_QOS_POLICY**"* |
| **PI-6** — `unknown` on the never-driven branch | **REFUTED as a value defect.** SC-4 **GREEN** — the branch already reports `unknown` + `STALE` correctly. It was **unasserted**, not wrong. It is asserted now. |
| **AoU-S1** — the filter cannot carry its fault on `Layer::isCurrent()` | **HOLDS TODAY.** SC-6 **GREEN**, armed as a tripwire: it goes RED the day the upstream `setCurrent` reordering lands, which is the signal to rewrite AoU-S1 rather than to fix the test |
| PI-3, PI-5, PI-7 and every line/quote citation | **CONFIRMED by source read** — each carries a file position checkable without building |
| **Regression check on the closed set** | **GREEN.** Full suite re-run — **33 gtest cases, 2 failures, both of them SC-1 and SC-3, i.e. only the new oracle.** No pre-existing case regressed. |

⚑ **PI-4 was the one finding derived from a specification rather than from source read on disk, and
it was published as `CONFIRMED-BY-SPEC / UNVERIFIED-BY-EXECUTION`. It is now the best-evidenced
entry in this table.** ⚑ **And execution found something the reasoning had not:** the
incompatibility warning is emitted **on the publisher's node**, not the subscriber's. An integrator
who follows `README.md:451` gets **silence in their own process**, while the diagnostic explaining
why appears in a log that is **ours**. The failure is silent exactly where it needs to be heard.

### What remains unexecuted, and stays UNVERIFIED rather than *cleared*

| item | status |
|---|---|
| **The rmw generalisation for PI-4** | ⚑ **UNVERIFIED.** The result above is true of the rmw in this image. DEADLINE Request/Offered handling is implementation-specific; **a single-rmw answer does not generalise to your stack.** Verify on yours. |
| **The real-stack claim under the whole liveness argument** | ⚑ **NO LONGER UNVERIFIED — EXERCISED 2026-09-06.** Real `Costmap2DROS` + real `map_update_thread_` (5.000 Hz), nav2's own `controller_server`, target node **out of process**, genuine BT `ClearEntireCostmap` through `libnav2_clear_costmap_service_bt_node.so`. Harness at `hubot_live_stack/`, built by a build seat; `hubot` itself untouched. **Yielded PI-8 and PI-9.** ⚑ **Still UNVERIFIED within it:** no sensor layers, so filter-vs-obstacle-layer contention is unexercised; no goal sent, so the costmap thread is real but the LOAD is not; simulated TF; synthetic mask; single host, so cross-machine DDS is untested. Not a robot, not hardware. |
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
> This package had already cited it — `src/zone_parameter_filter.cpp:479`, written the day before
> this analysis — and the analysis **derived the class independently from four generations of a bug
> without ever resolving it.** Both routes reach the same place; only one of them is cheap.
>
> ⚑ **That is the whole lesson of *"learn from the visions."* A seat that goes to the genba and a
> seat that reads the corpus both arrive — but the seat that reads first arrives on turn one and
> spends the rest of its budget on what the corpus does NOT already know.** §3 is renamed in
> substance, not in text: **PI-CLASS is Vision 14 applied to a reporting component.**
>
> *(Attribution verified this turn with `--verify`; a false attribution to the same vision was run
> as a negative control and refused. The citation at `cpp:479` is a bare number with no text — it
> survives only because the surrounding sentence happens to paraphrase it. **FSE does not hold the
> pen on `src/`; named for CPP, not edited.**)*

| vision (verbatim) | the promise it makes of hubot | what keeps it |
|---|---|---|
| **14** — *"Silent failure is the anti-Jidoka — a function that returns a success-shaped value while the operation failed is a loom weaving through a broken warp."* | ⚑ **The root promise. `enforced` must never be success-shaped when the operation did not succeed — or was never attempted.** | **The entire table.** SC-1, SC-3, SC-5 RED · SC-2, SC-4, SC-6 GREEN · six closed generations · `CPP_N3`, `FSE_F`, `MF-1`, `MF-6`. ⚑ **This is the promise the package exists to keep, and it is the one it has broken five times.** |
| **9** — *"The operator must not be the last line of defense against defects — the machine itself must catch them."* | An integrator must not have to **notice** that a zone went unenforced. The component must say so. | **SC-1** (RED — it does not, with the detector off) · ⚑ **and this vision is what found PI-C2 and PI-C5: six entries marked closed, two with no oracle at all.** "Closed by a code change" makes the *reader* the last line of defence. |
| **88** — *"SRE pages on SLO burn — the production system self-reports when it has departed from contract, so humans investigate rather than stand watch."* | ⚑ **This is PI-CLASS stated positively.** hubot exists to self-report departure from contract. The defect family is that it self-reports **conformance** when nothing is looking. | **SC-1, SC-3, SC-5** (all RED) · **SC-2, SC-4** (GREEN — it *does* report correctly when the mechanism is live). The whole table is one question: does this system self-report departure, or only self-report? |
| **20** — *"Stopping must be cheap, or operators will hesitate; design the halt to cost less than the defect."* | A failed parameter set must **degrade and keep navigating**, never kill the node. | **Kept, and it is the package's reason for existing** — quoted in its own source at `cpp:770` and `cpp:819`. Oracles: `test_degrade_not_abort`, `HUBOT_A/B`, `survival_harness_test`, all **GREEN**. |
| **77** — *"Genchi genbutsu — go to the actual place and see the actual thing; second-hand descriptions lie."* | Findings about this component must come from the component, not from reading about it. | ⚑ **BROKEN BY THIS DOCUMENT'S OWN AUTHOR AND REPAIRED IN §6** — "the package cannot be built here" was written from `ls /opt/ros` on the host while docker, the image and a built workspace were all present. **Now kept**: every finding above was executed. ⚑ **AND NOW KEPT ON THE SECOND LIMB TOO**: a build seat stood up a real `Costmap2DROS` with its own `map_update_thread_` at a measured 5.000 Hz, nav2's own `controller_server` binary, the target node in a **separate OS process**, and a genuine BT `ClearEntireCostmap`. ⚑ **Going there immediately produced PI-8 and PI-9 — two findings the fixture was STRUCTURALLY incapable of surfacing.** The promise paid the moment it was kept. |
| **45** — *"Invention that does not reach the market is not yet complete — commerce is how service lands on the user."* | Whatever this analysis establishes, it establishes about something **nobody can install**. | ⚑ **NOTHING KEEPS THIS, AND NOTHING HERE CAN.** The completing act is an upstream release, not ours. Stated on this document's face in §0 rather than buried, because it qualifies every row above. |

**How to read this ledger, and it is the point of it.** ⚑ **Three of the five promises were kept
only after something failed** — Vision 9 caught two unguarded entries, Vision 77 caught a false
sentence in this document, Vision 88 named the class the whole table is about. **A promise that has
never cost its holder anything has not been tested.** Vision 20's row is the one to be least
comfortable about: it is marked kept, its oracles are green, and **nothing on this page challenged
it this session.**

---

## 8 · Scope, provenance and audit status

Produced by **FSE** under a QM/advisory ceiling. **FSE is a producer and is not its own auditor**
— this document is unaudited and routes to **AAA + DIA**. It **attaches to** existing records and
re-authors none; measured this turn, **no `SAFETY_BOUNDARY.md` exists for `hubot`**, so nothing
was displaced.

Governing thesis: `D-VGC264-1` — *hubot is a bridge between robot output and human sensor.*
The insufficiency class above is a **robot→human** defect: the rate-reduction direction, where the
compression is required to preserve the decision and instead preserved a reassuring word. It says
nothing about the human→robot direction, which `D-VGC264-1` §3 measured as absent and named open.
