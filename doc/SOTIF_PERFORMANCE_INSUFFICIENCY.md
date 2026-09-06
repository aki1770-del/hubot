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
| **PI-1** | **The silence detector can be configured inert, and the component then makes an affirmative liveness claim instead of withholding one.** | `costmap_silence_timeout <= 0` — **documented** at `README.md:440` as *"keep the heartbeat but never report a stopped costmap"* — **and** the costmap subsequently stops. | `watching: yes`, `enforced: yes`, level `OK`, *"Zone N is in force."*, republished every `liveness_period`, indefinitely. | The integrator asked **not to be told** about a stopped costmap. They were given a **positive claim that it is running.** This is generation 3 restored by one line of YAML, past a value-level gate built to prevent it. | `cpp:923-924` (short-circuit) · `hpp:333` (`notWatching()`) · `cpp:1080` (`watching` derives from it) · **`grep -rn "silence_timeout" test/` → only `2.0` and `0.3`; the `<= 0` path has ZERO tests** | ⚑ **OPEN — LIVE** |
| **PI-2** | **The declared expiry exceeds the publisher's own threshold for calling itself stale, and neither number has a ceiling.** | Shipped defaults. No misconfiguration required. | `valid_for_s: 2.5` while `costmap_silence_timeout` is `2.0`. | `README.md:248` instructs: *"If `now - header.stamp > valid_for_s`, stop believing the message."* Read literally — and it is written to be read literally, *"by three lines of consumer code"* — the consumer keeps believing an `enforced: yes` for **2.5 s** after the last message, while the publisher would have disowned its own reading at **2.0 s**. Widens without bound: `liveness_period: 30.0` → `valid_for_s: 75.0`, and on a moving platform that is a distance, not a time. | `cpp:59` (`kValidForPeriods = 2.5`) · `cpp:1086` (product) · `hpp:467` (`2.0` default) · `README.md:248` · **no clamp on either parameter anywhere in the translation unit** | ⚑ **OPEN — LIVE** |
| **PI-3** | **`valid_for_s` bounds the freshness of the REPORT; it does not bound the validity of the CLAIM, and the two are presented as one number.** | Any third party sets the same parameter on a target after this component's confirmation. | A message inside its `valid_for_s` window carrying `enforced: yes` for a value that is no longer set. | The component's own AoU-4 already states that a confirmed set means *acceptance*, not *still holding* — **unbounded in time, unverified by construction.** So the claim can go false at any instant, with no bound at all, while the only number a consumer is given to reason about expiry is a **publisher-liveness** number. A consumer told *"believe it for `valid_for_s`"* will infer the converse. The converse is not warranted. | `hpp:590-594` (AoU-4) · `cpp:1074-1079` (the field's own rationale, which is about node death only) | **OPEN — inherent; disclosure is the countermeasure** |
| **PI-4** | ⚑ **The only complete countermeasure the component recommends is structurally incompatible with the component's own publisher.** | An integrator follows `README.md:451` and requests a finite `DEADLINE` QoS on their `zone_decision` subscription. | Under DDS Request/Offered matching, an **offered** deadline of infinity does not satisfy any **finite** requested deadline. The subscription does not match; the consumer receives **nothing at all** — which they are most likely to read as a broken topic, not as *"not watching."* | `decision_pub_` is created with a bare `rclcpp::QoS(10)` (`cpp:147`), leaving the offered deadline at the rmw default (infinite). The package tells the integrator to build the one mechanism it prevents them from using. **One-line fix on the publisher side:** offer a deadline derived from `liveness_period`. | `cpp:147` · `README.md:450-454` · `hpp:633-636` | **OPEN — `CONFIRMED-BY-SPEC`, ⚑ `UNVERIFIED-BY-EXECUTION`; see §6** |
| **PI-5** | **Detection latency is bounded by `liveness_period`, not by the `costmap_silence_timeout` an integrator sets to express their budget.** | `liveness_period > costmap_silence_timeout` — e.g. `0.5` / `5.0`. | Silence declared after up to `costmap_silence_timeout + liveness_period`, not `costmap_silence_timeout`. | The parameter named for the budget does not govern the budget. No consistency check exists between the two; the startup warnings at `cpp:113-128` fire **only on `<= 0`**, so a self-defeating pair is accepted in silence. | `cpp:922-925` (age evaluated once per tick) · `cpp:113-128` (warning coverage) | **OPEN — LIVE** |
| **PI-6** | **The `unknown` vocabulary is asserted positively on one of the two branches that produce it.** | A future change alters the never-driven branch's token. | `MF-1` asserts three negatives (`!= yes`, `watching != yes`, `level != OK`); a change emitting `NO` there passes. | The `unknown` / `NO` distinction is load-bearing and the component argues it at `cpp:1003-1010` — STALE not ERROR, because *"nothing was measured and found bad; nothing was measured."* `NO` sends an operator to look for a fault that does not exist. A vocabulary argued this carefully needs a positive oracle on every branch. | `test/maintainer_findings_test.cpp:410-431` (MF-1, negatives) vs `:640-666` (MF-6, positive, stopped branch only) | **OPEN — narrow; SC-4 closes it** |
| **PI-7** | **State 0 reports nominal defaults *in force* having issued nothing, when `nominal_defaults` is empty.** | `nominal_defaults` declared empty; robot leaves the mask. | level `OK`, *"Outside any zone; nominal defaults are in force."* — zero sets issued, zero confirmed. | A positive claim about a restoration that did not occur. **Partially mitigated:** a per-parameter config-load warning fires when a state overrides a parameter with no nominal entry (`cpp:352-368`). It does not fire for the wholly-empty case. | `cpp:683-688` (`resetToNominal` iterates an empty map) · `cpp:1036-1038` (the claim) | **OPEN — low reach; disclosure sufficient** |

### Closed, retained so a reader can see the shape repeat

| # | insufficiency | closed by |
|---|---|---|
| PI-C1 | `enforced: yes` on issue | `unconfirmed_targets_` → `pending` (`cpp:895`) |
| PI-C2 | `enforced: yes` at startup | `state_initialized_` gate (`cpp:960-963`) |
| PI-C3 | `enforced: yes` during silence | liveness timer + `unknown` (`cpp:903-945`, `:892-894`) |
| PI-C4 | `enforced: yes` before the first tick | `notWatching()` in the value (`hpp:333`) |
| PI-C5 | silent unenforcement when no client exists | `markTargetDegraded` on missing client (`cpp:728-736`) |
| PI-C6 | a set that never answers reported as success | `set_parameters_timeout` deadline (`cpp:785-796`) |

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

> **AoU-S6 — SILENCE ON `zone_decision` IS *"I AM NOT WATCHING"*, AND THE NODE'S DEATH IS OUTSIDE
> THIS COMPONENT.** If the node dies the timer dies with it and the last message stands. Nothing
> running inside a process can announce that process's own death. **The complete answer lives in
> your process and is not built here.**

---

## 6 · ⚑ What is UNVERIFIED, and stays UNVERIFIED rather than *cleared*

**Nothing in this analysis was executed.** Measured this turn: `/opt/ros` does not exist on this
machine, `colcon` is not installed, and no released `nav2_costmap_2d` carries the
`ZONE_PARAMETER_FILTER` constant this package's source names — the package's own headline
disclosure. **So the whole component cannot be built here, and every finding above is from reading
the source, not from running it.**

| claim | class |
|---|---|
| PI-1, PI-2, PI-5, PI-6, PI-7 and every line/quote citation | **CONFIRMED by source read** — each carries a file position a reader can check without building |
| PI-2's arithmetic (`2.5 > 2.0` at defaults) | **CONFIRMED** — it is arithmetic over two literals, `cpp:59` and `hpp:467` |
| **PI-4** — DDS DEADLINE Request/Offered incompatibility | ⚑ **CONFIRMED-BY-SPEC, UNVERIFIED-BY-EXECUTION.** The RxO rule is from the DDS specification, not from an observation on this machine. **Falsified if** a subscriber requesting a finite deadline receives messages from this publisher on the integrator's rmw. Test it before you rely on either the finding or the README's advice. |
| `test/sotif_gate_inertness_test.cpp` SC-1 / SC-3 predicted RED | ⚑ **PREDICTION, NOT MEASUREMENT.** Every other suite in this package records a RED-before / GREEN-after. **This one records none and must not claim one.** |

**The test is registered in `CMakeLists.txt` deliberately though it has never run** — an
unregistered test is a claim rather than a feature, which is the defect this package recorded
about its own static lane. **It must be run, and its verdict recorded, before any of it is
believed.**

---

## 7 · Scope, provenance and audit status

Produced by **FSE** under a QM/advisory ceiling. **FSE is a producer and is not its own auditor**
— this document is unaudited and routes to **AAA + DIA**. It **attaches to** existing records and
re-authors none; measured this turn, **no `SAFETY_BOUNDARY.md` exists for `hubot`**, so nothing
was displaced.

Governing thesis: `D-VGC264-1` — *hubot is a bridge between robot output and human sensor.*
The insufficiency class above is a **robot→human** defect: the rate-reduction direction, where the
compression is required to preserve the decision and instead preserved a reassuring word. It says
nothing about the human→robot direction, which `D-VGC264-1` §3 measured as absent and named open.
