# Changelog

All notable changes to `hubot`. Format follows Keep a Changelog; versions follow SemVer.

## [Unreleased]

### Added
- ⚑ **THE BRING-UP. `launch/`, `params/`, `params/examples/` and `maps/` — the four things
  the package told the integrator to build for herself.** A nav2 costmap filter needs five
  things before it can run at all: the plugin, a filter mask, a `map_server` publishing it, a
  `costmap_filter_info_server`, and params plus a launch file wiring them together. Until now
  this package shipped **the first only**, and the page said so as if it were a bound rather
  than a gap. `ros2 launch hubot zone_filter_demo_launch.py` now brings the filter up inside
  nav2's own `Costmap2DROS` and reaches `enforced: yes`, with everything resolved out of the
  **install space** — the launch file reads `get_package_share_directory('hubot')` and can
  reach nothing in a source tree.
  - **It ships two CONTROLS, not a demo.** `overlay_target_readonly.yaml` (the target refuses
    the set) and `overlay_target_inside_namespace.yaml` (the set never arrives) both end in
    `enforced: NO`, for different reasons, with different sentences. A value with no control
    beside it is not evidence — the package's own argument, now applied to its own demo.
  - ⚑ **The second control was WRONG when written and the run is what caught it.** It first
    named a ROOT target relatively and claimed that reproduced a namespace miss. Measured
    2026-09-06: `enforced: yes`, `arrivals=1`. The relative-name case is **cured** on this
    tree — `joinWithParentNamespace()` maps it to `/zone_target_demo` — and a control that
    asserts a failure the package already fixed teaches a defect that no longer exists and
    makes a working package look broken. Replaced by the case that IS live: **PI-16's mirror**,
    a target genuinely inside `/local_costmap` which can no longer be named relatively at all.
    Measured: `enforced: NO` naming `/costmap`, the phantom node the join produces, while
    `ros2 param set /local_costmap/costmap robot_radius 0.5` externally succeeds. **PI-16 is
    now not merely disclosed in prose; it is reproducible in one command.**
  - **The mask is PLAIN TEXT, and it is its own documentation.** `maps/zone_mask.pgm` is an
    ASCII `P2` PGM: checked in, so nothing has to be generated; readable, diffable and
    editable in any text editor, so it is not the binary-in-the-tree defect either. nav2's own
    example masks are GIMP-written `P5` binaries; this one loads through the same `map_server`
    and a reader can see every zone id. Verified by running it, not by reasoning about it.
  - **`nominal_defaults` is set from the launch file and this is the first form ever measured
    to load.** README warning 1 — that the block cannot be written as nested yaml and the
    dotted form is untested — is unchanged for a *parameter file*. As launch-file node
    parameters the dotted names load and are consumed: `1 nominal default(s) loaded for
    state-0 reset`, up from `0`, and the out-of-zone case now actually restores (`arrivals`
    0 → 1) instead of reporting `enforced: yes` for a state that put nothing back.
  - **`src/zone_target_demo_node.cpp`** — the node the filter talks to. hubot does not set a
    variable; it sends `set_parameters` to another process. A bring-up with no target cannot
    reach `enforced: yes` at all, so shipping one is the precondition for the falsifier, not
    scope creep. On a robot this is `controller_server` and `FollowPath.max_vel_x`.
  - **`nav2_map_server` and `nav2_lifecycle_manager` are declared `exec_depend`**, runtime-only:
    the filter library links neither and building does not need them. An integrator who wants
    only the plugin `.so` can take the build dependencies alone.
- ⚑ **`prose_matches_tree.py` CHK-6, CHK-7, CHK-8 — because a warning did not work, three
  times, in one sitting.** `package.xml` carries a comment saying in its own words that XML
  forbids a double hyphen inside a comment and that an earlier draft *"used the flag's real
  spelling and made package.xml unparseable"*. The next person to edit that file broke it the
  same way **three times on 2026-09-06**, twice within twenty lines of the warning. A rule
  that is written, read, and even quoted back still does not fire; only a check the author
  cannot decline does.
  - **CHK-6** every shipped XML and Python file parses (stdlib only, so the toolchain-free
    lane keeps its promise). **CHK-7** every shipped `.yaml` parses, reporting UNVERIFIED
    rather than GREEN where `pyyaml` is absent. **CHK-8** every directory that exists in the
    tree is named by an `install()` rule — the difference between *committed* and *shipped*,
    which otherwise costs a container run to discover.
  - All three carry negative controls in `--selftest`, and **CHK-6's control is the defect
    itself**: it inserts a double hyphen into a `package.xml` comment. 14 of 14 controls prove
    their check behaves as claimed.
- ⚑ **`test/namespaced_target_resolution_test.cpp` — the fixture that was recorded as
  structurally impossible, and was not.** The relative-`node:` finding was measured only in a
  real multi-process stack and written up as *"structurally invisible to every in-process
  fixture, which hosts the target at root namespace where a relative name resolves correctly
  BY ACCIDENT."* That sentence is true and was read as meaning the case needed a robot.
  **It needed one line.** The variable is not where the TARGET lives — it is where the HOST
  lives: put the filter's own node in `/local_costmap`, leave the target at the root exactly
  where every other suite already puts it, and the accident is gone. Everything else keeps
  working under the namespaced host for the reason that makes this a single-variable
  experiment: the filter joins its own four topic names to the **parent** namespace, and the
  parent of `/local_costmap` is the root, so info, mask and decision land byte-identically to
  the un-namespaced fixtures. **Case A asserts the resolved client key and fails on the tree
  before the join** (the key was the raw declared string); **case B asserts the set actually
  arrives**; **C and D are a PAIR** differing in exactly one thing — whether anything exists at
  the name — so removing the discrimination turns one of them red. Every case asserts the
  namespace premise first, because a fixture whose namespace silently did not take would pass
  all four for the wrong reason.
- ⚑ **`prose_matches_tree.py` SC-11 — the operator sentences SC-9 was never reading.** SC-9
  scans `publishDecision()`'s branch literals and nothing else. The `why` on a degraded target
  is carried to the operator on the `event` field from three other functions, and the longest
  and most directive sentence in the package — the one that says walk to this node, or do not
  — was written **outside every oracle in that file on the day it was authored.** SC-11 applies
  SC-9's rule to those three functions. It **refuses to pass on an unread surface**: if the
  functions are renamed it reports that it could not be evaluated rather than green over
  nothing. Its negative control mutates a literal that exists only inside
  `describeUnansweredTarget()` — the anchoring lesson SC-9's own first control taught this file
  — and was **watched going red**: 10/10 controls now pass.

- **`test/prose_matches_tree.py` — the prose is now re-derived from the tree, and disagreeing
  fails.** Four figures about this package's own tests were published at once and no two agreed
  (`28 of 28`, `23 of 26`, `35 tests`, and a `46` that was `colcon test-result --all` counting a
  different quantity without saying so); the README cited three `throw` line numbers that had all
  moved; and `doc/SPEC_COVERAGE.md`'s one-line summary contradicted a measurement printed twenty
  lines above it in the same file. **Every one was true when written.** Nothing re-derived any of
  them, so each went false at the next edit — one twice in a single day. Six checks: throw sites,
  the tag/`package.xml`/CHANGELOG triple, the canonical test counts, retired operator phrasing
  still published, and FSE's two message oracles. ⚑ **It ships its own negative controls**
  (`--selftest`) which mutate a copy of the tree once per check and require each to go RED.
  **The controls earned their keep immediately: SC-9's first control reported STILL GREEN**,
  because it mutated the first occurrence of a string literal that turns out to live in a comment
  at `cpp:648` rather than the code at `cpp:1268`. An oracle nobody has watched go red is not
  evidence that it can.

### Removed
- ⚑ **`doc/SPEC_COVERAGE.docx` — a tracked binary whose front page still said the package
  would not build, five weeks of corrections after it did.** Measured by extracting
  `word/document.xml` rather than searching the file: **1** occurrence of *"THE PACKAGE AS
  COMMITTED WILL NOT BUILD, AND THIS IS THE HEADLINE"*, dated `2026-09-05`, and **0** of
  `SUPERSEDED`, `IT BUILDS` or `2026-09-06` — while `doc/SPEC_COVERAGE.md:8` has carried
  *"SUPERSEDED 2026-09-06. IT BUILDS"* since the day it became true. ⚑ **A text search over
  a ZIP returns 0 for every pattern, which reads exactly like a clean file**, so an audit that
  searched it as text cleared it: the conclusion was wrong and the warrant was absent
  independently of the conclusion.
  - **Its reach, measured rather than assumed:** `CMakeLists.txt:77-80` installs the target,
    `include/` and `hubot_plugins.xml` and **not `doc/`** — so this never reached a consumer of
    the built package. It reached **anyone who cloned**, which is the reader this repository is
    about to acquire.
  - **Dropped, not regenerated, and the choice was available** — `pandoc` is on the host. Three
    measurements decided it: **nothing references it** (0 hits across every text format in the
    tree), and at **5,426 characters against a 158-line markdown** it was never a copy but a
    partial snapshot, so *"regenerate it faithfully"* has no defined target. And nothing would
    stop it rotting again at the next edit of the markdown except a second oracle that extracts
    and diffs a document with no named reader. **This is the second time in one day a citation
    rotted because two copies of one document existed.** The markdown is the source of truth.
  - ⚑ **The loom is not "regenerate it correctly once" — it is CHK-5, which makes the gate's
    reach equal its claim.** `doc_files()` globs `.md` and nothing else, so every prose check
    was blind to any other format while reporting green. CHK-5 refuses to let a document exist
    under `doc/` that the gate cannot read. Its control has to **create** the offending file
    rather than mutate a string, because the defect is a file that exists — which is exactly why
    no text mutation ever caught the real one. **CHK-5 was watched going RED on the actual
    `.docx` before it was removed**, not only on the synthetic control; 11/11 controls pass.

### Fixed
- ⚑ **THE COMPONENT NAMED A HEALTHY NODE, AND ONLY HALF OF THAT IS FIXED BY THE NAMESPACE
  JOIN. Measured before anything was changed, and the answer came back split.** The reported
  instance — a relative `node:` under a namespaced costmap — **is closed.** Measured at nav2
  tag `1.5.1`: `Layer::joinWithParentNamespace()` strips one level
  (`node_namespace.substr(0, node_namespace.rfind("/"))`, `layer.cpp:90-93`, read at tag `1.5.1`), so from a filter
  hosted by `/local_costmap/local_costmap` a relative `zpf_target_node` now resolves to
  `/zpf_target_node` — the node that was reachable the whole time. Cases A and B of the new
  suite hold that, and both fail on the tree before the join.
  - ⚑ **The CLASS is not closed, and the fix opened a fresh instance of it.** "no answer within
    Ns of the set being issued" describes a target that is present and did not reply. It was
    also the sentence for a target that **does not exist at all**, and those are opposite
    instructions to a person: one is a node to walk to, the other is a line of YAML to correct.
    Nothing on the wire separated them — the finding says so itself, that only a pending-record
    count of 7 versus 1 distinguished refused from never addressed. And a name that was
    previously resolving *inside* the costmap's namespace and working now resolves one level up
    and does not, producing the same misleading sentence from the opposite cause.
  - **Closed in the value.** `describeUnansweredTarget()` asks the client whether a parameter
    service is actually there and says which case this is, in the log **and** on `event`. When
    the join moved the name, the sentence carries **both** spellings — the report says
    `/zpf_target_node`, her YAML says `zpf_target_node`, and connecting those two is the whole
    repair. ⚑ **It reports what it OBSERVED, never that the node is absent**: discovery is
    asynchronous, and claiming absence would replace *names a healthy node* with *declares a
    live node dead* — the same defect mirrored. Where two declared names resolve onto one
    target the mapping is **erased rather than guessed**; naming the wrong YAML line is worse
    than naming none.
- ⚑ **OUR OWN ACCOUNT OF THAT DEFECT WAS BACKWARDS, IN THE SOURCE AND IN THIS FILE.** Both said
  the relative name *"resolved against the root while our topics resolved against the parent."*
  **That is refuted by its own evidence**: resolving `controller_server` against the root gives
  `/controller_server`, which is the intended node, and there would have been no defect to fix.
  Measured instead — rclcpp builds `<remote_node_name>/set_parameters`
  (`parameter_client.cpp:74`) and hands that **relative** name to `rcl_node_resolve_name`
  (`client.c:123`), which expands it against **the owning node's** namespace. It resolved
  against the CHILD, not the root. The fix was right and the reason was wrong, **and a wrong
  reason is how the next person rebuilds the defect.** Corrected in both places; the README's
  description at `:228-231` was already correct and is untouched.
- ⚑ **Three classes of citation a reader of this repository cannot follow.** A pointer to
  `CLAUDE.md` — **a file that does not exist here** — in three test files; an internal rule
  ordinal (`OPS-070(B)`) in five; an internal ordinal `V15` in three; an unattributable quote
  from an unnamed authority in the source; and nine bare corpus numbers (`Vision 14`, `20`,
  `77`, `9`). In every case **the sentence was self-contained and only the citation was
  unfollowable**, so the sentence stays and the citation goes. Sakichi's principles are now
  **quoted in full** where they are relied on rather than cited by a number, which is the rule
  this project already applied to the document that ships outside the repository: *a citation
  the reader cannot resolve is worse than none, because it looks resolved.* The three-letter
  author tags in the comments are not deleted — they are the authorship record — but README now
  says what they are and that **nothing outside this repository needs to be looked up.**
- ⚑ **A test colour was stated as present tense in three documents and one of them did not
  agree with itself.** `CMakeLists.txt` carried *"SC-1, SC-3 and SC-5 RED … 33 cases, 2
  failures"* — **three named red, two counted** — and all three cases were worked on afterwards
  with nothing coming back to update the comment. Each claim is now scoped to the run and tree
  it measured. **No colour is asserted or retracted here**: that is a safety-oracle verdict and
  belongs to FSE and to a run, not to the surface that quotes it.
- ⚑ **`12.000000s` was reaching the operator, and it reached a tag that way.** `costmap_age` and
  `set_parameters_timeout` went through `std::to_string(double)`, which is `"%f"` — six decimals.
  It survived because **every internal quotation of the sentence tidies the float away**
  (`doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md:139` elides it), so the wire text had never once been
  read as it ships. New `humanSeconds()` gives one decimal in the sentences a person reads.
  **Deliberately NOT applied to the `KeyValue` fields** — those are parsed with `std::stod` by
  consumer code and by this suite against tight thresholds, and rounding at a boundary would
  change a machine's verdict to buy a human nothing.
- ⚑ **The two `NOT WATCHING` sentences claimed more than the filter measured — a failure-shaped
  value overstating danger, the inverse of the defect this package exists to abolish.** *"Decide
  as if nobody is watching, because nobody is"* asserted a fact about the safety scanner, the
  bumper, the E-stop, the controller's own collision checking and the person in the doorway. This
  filter measured exactly one thing: **it** has not been driven. Both branches now say so and
  stop there. Both also opened with a subjectless *"NOT WATCHING."* — beside a robot in motion
  the subject a reader supplies is the robot; the subject is now written down.
- **The loudest imperative fired on every ordinary bringup.** The never-driven branch publishes
  at 1 Hz from configure, before the costmap update thread exists, and carried an imperative of
  the same force as the one that fires an hour after the costmap died. An imperative that fires
  when nothing is wrong teaches an operator to discount the one that fires when something is.
  The standing is now differentiated in the sentence.
- **`"Do not rely on its limits until this reads yes"` was an unbounded wait**, published beside
  a moving robot. It now names the deadline that actually exists (`set_parameters_timeout`,
  swept by `checkPendingParameterUpdates()`) — **and says the other thing when that deadline is
  switched off**, which is a documented configuration under which `pending` genuinely never
  resolves. Naming the parameter unconditionally would have been unsatisfiable in precisely the
  case the sentence was written for.
- **`"on at least one target"` now names the target.** The name was sitting in `event`, behind a
  click, while the sentence in front of the operator declined to say it.
- **`"what is in force is still zone N's"` asserted another process's current parameter state**,
  which this filter cannot read. AoU-4: a confirmed set means acceptance, not that it still
  holds. Now *"what we last confirmed was zone N's."*
- **`README.md` told the reader the opposite of what the component does.** *"An `OK` from ninety
  seconds ago renders identically to an `OK` from now"* stood in the present tense; measured on
  this tree, silence past the budget sets `STALE`, not `OK`. ⚑ **And the residual is not the one
  you would guess** — `costmap_silence_timeout: 0` does not restore it, because the budget falls
  back to `liveness_period x 2.5`. The window that genuinely survives is the node dying outright.
- **Stale citations on shipped surfaces**: the three `throw` sites (`:78`/`:182`/`:251` →
  `:105`/`:270`/`:339`), `doc/SPEC_COVERAGE.md`'s *"Two of the six"* against its own *"Three of
  six"* twenty lines later, its *"Build: never attempted"* against its own green build, and
  `CMakeLists.txt`'s unsourceable *"486 passing tests"* — removed rather than replaced with a
  guess, because the mechanism carried the argument on its own.
- **`test/maintainer_findings_test.cpp` — a harness built to the specification of a review we
  already received.** nav2's maintainer found something on all three rounds of review of the
  upstream sibling of this filter, and asked, plainly, *"which maybe you can catch yourself? Not
  sure why I'm catching them but you're not."* His findings are the specification here.
  Six cases. **Four failed on the parent commit before any fix existed** and are recorded with
  that output; one is a negative control that must stay green; one is coverage for a breaking
  change that had none.
  - ⚑ **Why 486 passing tests caught none of them: every one of his findings is about a
    SEQUENCE — a lifecycle transition or a message arrival order — and not one is "this function
    computes the wrong value."** A value oracle cannot fail on a sequence defect.
  - ⚑ **And why our own suite could not have caught them either: the setup helper waited the
    defect out.** It called `resetFilter()` and then spun until `isActive()`, which is
    `filter_mask_ != nullptr` — so it blocked until the latched mask had been redelivered, which
    is precisely the window his blocking findings live in. **The new fixture has no
    wait-for-active after a reset**; it calls `reset()` (the production entry point) and asserts
    immediately, inside the window.
- `doc/SPEC_COVERAGE.md` §4 now carries **a per-finding verdict with its evidence** — occurs
  here / cannot occur here with a named reason / unverified — because *"does not apply here"* is
  the sentence under which a defect survives a sweep.

### Fixed
- ⚑ **The message promised the human more currency than the robot's own doubt allowed.** At the shipped defaults `zone_decision` declared `valid_for_s: 2.5` (`liveness_period x 2.5`) while the filter called its own reading stale at `costmap_silence_timeout: 2.0` — a half-second in which a consumer honouring the declared expiry acted on a claim the publisher had already disowned, and with no ceiling on either parameter, `liveness_period: 30` yielded a 75-second-old enforcement claim a consumer was instructed to believe. **The sixth generation of the reassuring-value family: yes-past-my-own-doubt.** `valid_for_s` and the offered `DEADLINE` are now one number from one function, `declaredValidForS()`, clamped to `costmap_silence_timeout` while that detector is on (with it off, the fallback silence budget already equals `liveness_period x 2.5`, so they are equal by construction). Fixed in the value, and SC-3 was left exactly as written rather than relaxed to meet the code.
  - ⚑ **THIS ENTRY AND THE PHASE-B ENTRY BELOW CONTRADICTED EACH OTHER ON SC-3, INSIDE THIS ONE FILE.** This entry said SC-3 was *"green on both substrates"*; the Phase-B entry below lists SC-3 among three cases that do **not** pass against release `1.5.1`. **Both were true of the tree each measured, and neither said which tree that was** — the Phase-B count was taken before the fix recorded here landed, and nothing re-derived it afterwards. Both are now scoped to their own measurement. ⚑ **The colour of SC-3 is not settled here and is not CPP's to settle**: it is a safety-oracle verdict and belongs to FSE and to a run. What CPP can state is the bound — `colcon` cannot run on the host these words were written on, so no arm of this was executed by their author. A clean-room run of commit `4bfd2cd` was **reported** to CPP as `47 tests, 0 errors, 0 failures, 2 skipped`; CPP did not run it, cites it as relayed rather than measured, and notes only that a skip count of 2 is consistent with the two pluginlib cases that `GTEST_SKIP()` on a release. **Three documents on this tree carried a colour for SC-3 and the third — a comment in `CMakeLists.txt` — did not agree with itself.**
  - ⚑ **A wrong configuration is made visible, never silently corrected:** when the clamp bites, one warning at startup names both numbers and the clamped value. **The shipped defaults trip it.** That is a value decision about the defaults, recorded here rather than hidden by quietly moving one.
- ⚑ **Two pluginlib cases skip with the reason on a release, and the suite gains the case it lacked.** `A_PluginlibResolvesAndInstantiatesTheClass` and `C_NegativeControl_UpstreamFilterEscapesUpdateMap` discriminate this package against **upstream's** `nav2_costmap_2d::ZoneParameterFilter`, loaded by pluginlib name. A released nav2 does not ship it — measured at tag `1.5.1`: declared by nothing on the install — so on a release those cases have no control. They now `GTEST_SKIP()` with exactly that reason: a skip that says why, never a pass manufactured by the absence of the thing under test. B still runs there; the skip is the record that its control did not.
  - **New: `A0_PluginlibResolvesHubotsOwnClassOnThisSubstrate`** — loads `hubot::ZoneParameterFilter` through pluginlib with no upstream dependency, and must be green on every substrate the package claims. FBR's finding, verbatim: *"pluginlib loadability of hubot's own class on the release is UNVERIFIED by this suite. Yesterday's dlopen is not pluginlib."* Case A asserted the same load but aborted on its upstream arm before that assertion could report.
- ⚑ **This package now builds against a RELEASED nav2.** For a day it could not, and every surface said so: the source imported `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, a symbol on branches `main`/`lyrical` and in no release tag. **Measured 2026-09-06 (Phase A): that symbol was the ONLY blocker, at eight sites — two in the library, six in tests — and the full nav2 dependency chain built from tag `1.5.1` with it substituted.** Three candidate second blockers — `nav2_ros_common`, `declare_or_get_parameter`, `joinWithParentNamespace` — were refuted by build, not by grep: all present at `1.5.1` and `1.5.0`.
  - **It was our copying mistake, not upstream's omission.** The value is a wire discriminator the integrator's own `costmap_filter_info_server` publishes from YAML; this filter only has to agree with it and never needed to obtain it from a header. It is now `hubot::kZoneParameterFilterType` (`include/hubot/zone_parameter_filter.hpp:64`), used at `src/zone_parameter_filter.cpp:238` and `:242` and in six test files.
  - ⚑ **The literal `4` from the Phase A probe did not ship.** Two `static_assert`s guard the value: `BINARY_FILTER + 1`, always on, fires on every nav2 if upstream renumbers its filters (`.hpp:68`); and a cross-check against `nav2_costmap_2d::ZONE_PARAMETER_FILTER` wherever CMake finds that symbol (`.hpp:79`). **Both fired on a mutated value, on both substrates, build exit 2** — release tripped the first, branch tripped both.
  - ⚑ **`__has_include` was the wrong tool, and it was CPP's own proposal.** `filter_values.hpp` exists on release AND on branch; only the branch defines the constant inside it, so a header probe guards nothing. CMake probes the SYMBOL (`check_cxx_source_compiles`, `CMakeLists.txt:61`) and defines `HUBOT_UPSTREAM_HAS_ZONE_PARAMETER_FILTER`; the configure log states which way it went.
  - **Building is not passing, so both are stated.** ⚑ **Measured at Phase A/B on 2026-09-06, BEFORE the fixes recorded above in this same release landed; the counts below describe that tree and were never re-derived.** Against release `1.5.1`: the library and every test binary build; **23 of 26 tests passed.** The three that do not: SC-3 (pre-existing, identical on branch), and `pluginlib_live_costmap_test` A and C, whose *upstream-comparison* arm loads `nav2_costmap_2d::ZoneParameterFilter` — a plugin that exists only on branch. **Our own plugin resolves and runs through a live `LayeredCostmap` on release** (B and B2 green). `1.5.0` was not built and is not claimed. RMW: `rmw_fastrtps_cpp`.
  - Reversed on every surface that said otherwise: the README install section (retitled in place, the one-line check kept, the compiler transcript kept as the record of what it was), the README deploy section, `package.xml`, `doc/SPEC_COVERAGE.md`, and the `[0.1.0]` note below. ⚑ `package.xml` was made unparseable **twice** today by a `--` inside an XML comment — the second time in a comment that replaced one warning against exactly that. Both repaired, both recorded.
- ⚑ **Our own page told an integrator to use a mechanism our publisher made impossible.** `decision_pub_` was created with a bare `rclcpp::QoS(10)`, leaving the OFFERED deadline at the middleware default of infinity, while the README instructed the reader to request a finite `DEADLINE` QoS on their subscription. **`DEADLINE` is a Request/Offered policy and an offered infinity satisfies no finite request**, so a person who followed our written instruction got a subscription that never matched and received **nothing** — which reads as a broken topic, not as *nobody is watching*. Of everything found this week it is the only defect whose harm was specified in our own documentation. The publisher now offers `liveness_period x kValidForPeriods`.
  - **The number is not chosen to make a test pass.** It is byte-identical to the `valid_for_s` already in the payload, so the QoS promise and the message promise are one promise. It is keepable: the heartbeat publishes every `liveness_period` regardless of change, giving 2.5x headroom, so one late or dropped timer fire does not breach it — the same tolerance argument that justifies `kValidForPeriods` where it is defined.
  - ⚑ **It is offered only when it can be kept.** With `liveness_period <= 0` the heartbeat is disabled and nothing publishes periodically, so no deadline is offered. A publisher that offers a deadline it cannot meet trades a silent failure for a permanent false alarm.
  - ⚑ **The fix creates a trap and the README now names it:** the consumer must request a deadline **no shorter** than ours. A stricter request — `1.0 s` against our `2.5 s` — silently fails to match. An `incompatible_qos_callback` distinguishes that from a dead publisher; without one it cannot be told apart.
  - **Verified by execution, not by specification.** SC-5 green, both arms, with its default-QoS control arm required to receive. **BOUND: `rmw_fastrtps_cpp` only** — QoS matching is RMW-dependent and this does not generalise. **No new message key: 13 before, 13 after.**
- ⚑ **We namespace-joined our own four topic names and none of the integrator's.** `Layer::joinWithParentNamespace()` was applied at `:130`, `:142`, `:147`, `:215` — every name we own — and to **zero** of the `node:` fields an integrator writes. Under a namespaced launch a relative `node: controller_server` therefore resolved against the root while our topics resolved against the parent; an `AsyncParametersClient` was built for a node that does not exist; and **the missing-client guard at `issueAsyncSetParameters()` could not fire, because the key it looks up is the same key the phantom was built under.** The set simply never landed. Their names now get the same loom as ours.
  - **Absolute names are unaffected** — nav2's `Layer::joinWithParentNamespace()` returns any name beginning with `/` unchanged (`layer.cpp:90-96`, read at release tag `1.5.1`), which is why every documented example kept working and the fault stayed invisible to us.
  - The join is deliberately placed **after** the empty-check: it maps `""` to `"<parent>/"`, which is non-empty, and would otherwise have silently defeated the guard that rejects a missing `node:`.
  - ⚑ The requirement appeared **zero times** in the README. That was part of the same defect and is fixed with it.
- ⚑ **The reason for a failure was published on exactly one message and then taken away.** `event` was gated on a non-empty `detail`, and the 1 Hz heartbeat passes `std::string()` on every tick where nothing flipped — so `target 'X' not enforcing: <why>` survived a single message over volatile transport while the condition it explained persisted, leaving `enforced: NO` and a non-zero `degraded_targets` with nothing saying why. The last non-empty reason is now carried forward. **No new field**: it is self-correcting, because every resolution path publishes its own non-empty detail which replaces it.
- ⚑ **One documented line of YAML reinstated the reassuring value.** `costmap_silence_timeout: 0` made `costmap_silent_` unreachable, so a filter whose costmap died an hour ago published `watching: yes`, `enforced: yes`, level `OK` forever. A fallback budget of `liveness_period x kValidForPeriods` now applies when the explicit detector is switched off. The explicit path is byte-for-byte unchanged for any positive timeout.
  - ⚑ **THE PROPOSED FIX WAS TRIED FIRST AND IS REFUTED BY MEASUREMENT.** `sotif_gate_inertness_test.cpp` SC-1 proposes, in its own comment, `notWatching() { return !ever_processed_ || costmap_silent_ || costmap_silence_timeout_ <= 0.0; }`. Implemented exactly as written it turns **SC-1 green and SC-2 red** — and SC-2 is that same file's non-optional negative control, which states *"if SC-2 ever goes red, SC-1 passing means nothing."* A constant disjunct cannot distinguish a driven costmap from a stopped one, so it buys SC-1 by making the filter never assert `yes` at all: the degenerate pass SC-2 exists to reject. **The gate was always right; its input was what could not move.** Fixed in the input; `notWatching()` is left at two disjuncts.
  - The startup warning promised the defect in words — *"`watching` will always read yes"* — and now announces the fallback budget instead.
- ⚑ **Two dependencies were `REQUIRED` by the build and declared in no manifest.**
  `nav2_util` and `nav2_ros_common` are both `find_package(... REQUIRED)` (`CMakeLists.txt:20`,
  `:21`) and both linked as exported targets (`:36`, `:37`, `:62`, `:63`), while
  `grep -c nav2_util package.xml` and `grep -c nav2_ros_common package.xml` **both returned 0**.
  `rosdep install --from-paths` installed neither, so the first thing a stranger saw was a
  configure error naming a package this manifest never mentioned. Both are now declared.
- ⚑ **Fifteen stale source line-numbers across four files — every prose citation in the package
  was wrong.** The source moved twice on 2026-09-06 and no citation followed it. Re-measured
  against the committed source and corrected: `ZONE_PARAMETER_FILTER` `:119`/`:123` → **`:198`/
  `:202`** (`README.md` ×2 sites, `CHANGELOG.md`, `package.xml`, `doc/SPEC_COVERAGE.md` ×2);
  the `nominal_defaults` string-array and child reads `:257`/`:260` → **`:337`/`:339`**; the three
  `Failed to lock node` throws `:48`/`:103`/`:172` → **`:78`/`:182`/`:251`**. ⚑ **`:48` had
  drifted onto a doc-comment line and `:257` onto an unrelated `target_node` read** — a citation
  that lands on plausible-looking code is worse than one that lands on nothing, because it
  survives a reader's spot-check.
  - ⚑ **The two pasted compiler transcripts were NOT renumbered, and that is deliberate.**
    Rewriting a line number inside a recorded tool output manufactures a transcript no compiler
    ever emitted. They are labelled instead with the commit that produces them — **hubot
    `f08bb1c`**, where the constant did sit at `:119` — so they are reproducible rather than
    merely disclaimed.
- ⚑ **The one gate written to run WITHOUT a toolchain was reachable only by HAVING the
  toolchain.** `test/safety_invariants_static.sh` states on its own face that it is deliberately
  toolchain-free because the behavioural suite cannot run without ROS and *"the suite did not
  run" reads exactly like "the suite passed."* Its only invocation was `add_test` at
  `CMakeLists.txt:133`, downstream of sixteen `find_package(... REQUIRED)` calls. **Measured
  2026-09-06 on a ROS-free host: cmake aborts at `CMakeLists.txt:14` — the first one, 119 lines
  above the registration — and writes no `CTestTestfile.cmake` at all**, so the gate was never
  registered, not merely skipped. The script also *claimed* `"This gate always runs, everywhere"`;
  it could, and it did not. The false sentence is corrected in place rather than deleted, the
  registration is kept for regression visibility, and the toolchain-free lane
  (`bash test/safety_invariants_static.sh .`) is now named in the README's install-blocker
  section — the one section a reader who cannot build is guaranteed to reach.
  **Verified this turn: 12/12 deterministic green, 6 invariants, no ROS on the host.**
- ⚑ **The package invited an install it cannot deliver, and the disqualifying fact was 220 lines
  below the invitation.** The README's `find_package(hubot REQUIRED)` block sat near the top with
  nothing between it and a reader; the release gap was recorded honestly, but in the bounds
  section near the end. **An accurate fact a reader reaches after acting is not a disclosure.**
  The gap now precedes the instructions, says plainly that the package cannot be installed today,
  and hands the reader a one-line check to run against their own `nav2_costmap_2d` rather than a
  claim to trust.
- ⚑ **`doc/SPEC_COVERAGE.md` marked *"installable without an upstream merge"* as **FIT**.** It is
  a **GAP**, and the evidence column proved the wrong thing: presence of a `pluginlib` export says
  nothing about whether the source compiles. The row asked whether an integrator can install this
  and was answered with whether we had packaged it.
- ⚑ **`enforced: yes` and `watching: yes` from a filter that had never been driven once.**
  `initializeFilter()` runs at configure and starts the heartbeat; the costmap's update thread is
  not created until activate. In that gap — **every ordinary bringup, and re-opened by every
  routine `ClearEntireCostmap`** — the filter published level `OK`, `enforced: yes`,
  `watching: yes`, having read no mask, applied no state and confirmed nothing. A consumer could
  not distinguish it from a zone confirmed on every target.
  - ⚑ **This is the fourth generation of one defect family, and the first three were all fixed
    at a CALLER:** yes-on-issue, then yes-at-startup, then yes-during-silence. Each fix went into
    a call site, and each new call site then arrived without it — `livenessTick()` was the new
    caller this time. **This one is fixed in the VALUE.** `notWatching()` is a single predicate
    that both `enforcementToken()` and the `watching` field derive from, so a future caller
    cannot compute around it.
  - The discriminator already existed: `ever_processed_` was written in three places and read in
    exactly one — inside a message string, never in the token or the level. It is consulted by
    the value now.
  - **Not-yet-driven and stopped deliberately report identically**, because to a consumer they
    are the same fact: nobody is watching.
- ⚑ **A routine clear was silent on the surface that exists to report it.** `resetFilter()`
  destroyed both publishers at the top and mutated every piece of state a reader depends on
  below — mask, configuration, current state, outstanding sets — so nothing announced the
  transition. **This is the ordering defect nav2's maintainer named on the upstream filter,
  present here independently.** `resetFilter()` now clears state, publishes one farewell
  describing the cleared state, and destroys the publishers last.
- ⚑ **`pending` resolved to `yes` without any confirmation ever arriving.** `resetFilter()`
  discards `pending_sets_` and `unconfirmed_targets_` while those requests are still on the wire,
  so the filter forgot sets it had issued and could never learn their outcome — and the token it
  published moved to the reassuring value by discarding the evidence that refuted it. Now covered
  by the not-watching gate, so a clear reports `unknown` rather than `yes`.
- The not-watching message no longer says *"Zone N's limits were last requested"* when nothing
  was ever requested; the never-driven case gets its own sentence, true in its own branch.
- ⚑ **`test/safety_invariants_static.sh` was wrong half the time, and nothing ran it.** It had
  been committed since 2026-09-05 and was referenced by no CMake target, no hook and no cron —
  so it had never fired. Run by hand on unchanged source that satisfies every invariant, it
  reported **FAIL on 6 of 12 consecutive runs.** Cause, probed directly: two checks ended in
  `| grep -q` under `set -o pipefail`; `grep -q` exits on its first match, SIGPIPEs the upstream
  `grep`, and `pipefail` promotes the resulting 141 to the pipeline status (`rc=141` on 7 of 12
  probe runs). ⚑ **The failure mode was inverted** — the more obviously the invariant held, the
  likelier it was reported violated. Both sites now count with `grep -c`, which consumes its
  input. **20/20 green on the true source; 5/5 red against each of two mutants** (one removing
  **both** code references to `kMaxPendingSets`, one removing `pending_sets_.clear()`). It is now
  registered with CTest, so it runs in `colcon test`.
  - ⚑ **Corrected 2026-09-06: that read *"the only code reference"* and there are TWO** — the
    guard clause at `src/zone_parameter_filter.cpp:744` and the message at `:749`. The count was
    already 2 at `030bd4d`, the commit that introduced the sentence, so it was **wrong when
    written, not merely stale**. It is corrected rather than deleted because the error is
    load-bearing on the control itself: **a mutant that removes only one of the two leaves INV-C
    GREEN — measured this turn** — so a reader reproducing the recorded control from its own
    description would have got a passing mutant and concluded the gate was blind. A negative
    control is only evidence if its description reproduces it.

### Changed

- ⚑ **BREAKING, SILENT, AND FOR THE OPPOSITE TOPOLOGY — disclosed here while it is still free to disclose.**
  The namespace join applied to the integrator's `node:` field fixed the case where a *relatively named*
  target sat outside a namespaced costmap. **It broke the mirror case.** `Layer::joinWithParentNamespace()`
  strips one level, so with a costmap in `/local_costmap` a relative `foo` now resolves to `/foo`, where
  `rcl_node_resolve_name` previously gave `/local_costmap/foo`. **A target inside the costmap's own
  namespace worked before the join and does not now.** It changes behaviour with no error and no warning:
  the set is issued to a node that may not exist, and the report says only that nothing answered.
  - **Migration**: if your `node:` names a target inside the costmap's own namespace, **write it absolute.**
    An absolute name is unaffected — `joinWithParentNamespace()` returns any name beginning with `/` unchanged.
  - **Why it is disclosed now**: nobody holds this package, so this costs one paragraph. After the first
    consumer exists it costs their debugging session. The package argues exactly this for its own breaking
    changes — the cheapest moment it will ever be.
  - **Raised by FSE as PI-16, measured from nav2 source, not by the author of the join.** ⚑ **The join
    predates the target-describing commit; a bisecting reader needs them separate.**
- **Four dated self-corrections moved here from `README.md`.** A correction addressed to a reader of the previous page — of whom there are none outside this project — is discipline for a reader who does not exist. The corrected statements stay on the page; what each used to say, and why it was wrong, lives here.
  - *Not-current channel* — the page said the channel was *"closed to a derived filter three ways, measured on released `lyrical`"*. There is no released `lyrical`; it is a branch, and releases are tags. And the package's own header had already retracted "closed three ways": `Layer::setCurrent(bool)` is **public** (`layer.hpp:147`, in the `public:` region `:61`–`:181`), so the capability exists in the base class and is erased by statement order in the derived one, not by an architecture. The retracted sentence had been restated on the page in the same commit that corrected the header.
  - *Consumer rule* — it read *"refuse to drive on `enforced: NO` or `pending`"*: a blacklist, which cannot be complete and fails open — a consumer coded literally against those two strings would have read the new `unknown` as permission. It is a whitelist: proceed only on `enforced: yes`.
  - *Verified-against bound* — it read *"builds green against ROS `lyrical` with `nav2_costmap_2d` 1.5.1"*. The tree it was green against declared `<version>1.5.0</version>` and carried **six locally modified nav2 files** nothing warned about — `layered_costmap.hpp`/`.cpp`, `footprint_subscriber.hpp`/`.cpp` (26 lines), `keepout_filter.cpp` (7), `nav2_util/src/path_utils.cpp` (30), two of them the production caller's own class, in a directory that was not a git repository. Re-verified from a clean workspace against upstream `lyrical` HEAD `6f23b11c` with no local patches. ⚑ That same correction also said *"against released 1.5.1 it does not build at all"* — true when written, **superseded the same day** by the release-build entry under Fixed.
  - *The maintainer's findings* — the bounds section implied they were simply not our subject. They were checked one by one against this source; the per-finding verdicts and their evidence are in `doc/SPEC_COVERAGE.md` §4.
- `valid_for_s`'s `2.5` factor is now a named constant with its justification, and the README
  **defines the field for a reader** rather than only listing it.
- The README calls `valid_for_s` **"a declared expiry"** rather than *"a promise with a number in
  it."* An advisory component may not make a promise about timing; the information is the same.
- ⚑ **Corrected on this package's own face, rather than quietly:** `doc/SPEC_COVERAGE.md` said
  *"none of the six is present in Hubot."* **Two of them are, one character-for-character**, and
  that sentence is exactly the shape that lets a defect survive a sweep. `GAP-0` — *"THE PACKAGE
  AS COMMITTED WILL NOT BUILD"* — is also superseded: every row of it is now false, re-measured
  green. Both are corrected in place with the old text kept, because the old text is the reason
  the new text exists.

- **A positive liveness signal, so that silence on `zone_decision` reads as "I am not
  watching" rather than "you are safe."** A wall timer on the node — deliberately not on
  the costmap update loop — publishes every `liveness_period` whether or not anything
  changed, and declares the costmap stopped after `costmap_silence_timeout` without a
  `process()` call. New `values`: `watching`, `costmap_age_s`, `report_seq`,
  `report_period_s`, `valid_for_s`.
- The timer also runs the pending-parameter-set check, so **the `set_parameters` deadline
  fires and `pending` resolves while the costmap is stopped.** Previously neither could.
- New parameters `liveness_period` (default `1.0` s) and `costmap_silence_timeout`
  (default `2.0` s). Either `<= 0` disables its part of the mechanism, with a startup
  warning naming what is given up — the same convention `set_parameters_timeout` uses.
- `DiagnosticStatus::STALE` is now emitted, for the not-watching condition only.
- `test/costmap_silence_liveness_test.cpp` — six cases whose oracle **distinguishes a
  stopped filter from a running-and-quiet one**, driving `updateCosts()` from one thread
  while the executor spins on another. Two of the six are **negative controls that require
  the original defect to reappear** when the mechanism is switched off; both were verified
  passing against the unmodified filter before the fix existed, and one case exercises
  `resetFilter()` concurrently with a live timer.

### Changed
- ⚑ **BREAKING (vocabulary): `enforced` has a fourth value, `unknown`**, meaning the
  costmap has stopped calling the filter — nothing failed, and nothing is being checked.
  Made now because this package has no remotes and no consumer can hold the old set.
- ⚑ **BREAKING (contract): the documented consumer rule is a whitelist.** It was *"refuse
  to drive on `enforced: NO` or `pending`"*; it is now **"proceed only on `enforced:
  yes`"**. The blacklist form fails open — a consumer coded literally against it would
  have read `unknown` as permission.
- A **disabled** filter now reports `watching: NO`. `CostmapFilter::updateCosts()` returns
  early when `enabled_` is false (`costmap_filter.cpp:128-130`), and `enabled_` is flipped
  by the base class's `<name>/toggle_filter` service (`:82-86`), so a disabled filter
  previously looked identical to an enforced zone. This door was not named in any prior
  bound.

### Fixed
- **AoU-5 and the matching README bound were false and are replaced in place, with the old
  text kept.** They said the result check runs from `process()` "and nowhere else, because
  a costmap filter owns no timer", and offered "check it" as the countermeasure —
  detection by the reader. `Layer::node_` is a live protected handle (`layer.hpp:186`) and
  `Costmap2DROS` always runs `map_update_thread_` separately from the thread its node is
  spun on (`costmap_2d_ros.cpp:314`; all three non-test construction sites in nav2:
  `controller_server.cpp:72`, `planner_server.cpp:78`, `costmap_2d_node.cpp:47`).

### Known limitation — stated, not closed
- **If the node itself dies, the timer dies with it and the last message stands.** Nothing
  running inside a process can announce that process's own death. Every message now carries
  a declared expiry (`report_period_s`, `valid_for_s`) and a monotonic `report_seq` a frozen
  clock cannot fake, which is strictly better than a bare header stamp — **and is still a
  check the reader must perform.** The complete answer is a `DEADLINE` QoS or a
  `diagnostic_aggregator` staleness rule **in the consumer's process** — and the publisher
  now offers a deadline of `liveness_period x kValidForPeriods`, the same number it publishes
  as `valid_for_s`, so a `DEADLINE` request can match (measured, `rmw_fastrtps_cpp`, SC-5).
  ⚑ This line read *"NOT IMPLEMENTED HERE AND NOT MEASURED"* for a turn after that was no
  longer true. The consumer must still request a deadline **no shorter** than ours.

## [0.1.0] - 2026-09-06

### Added
- `hubot::ZoneParameterFilter` — a `nav2_costmap_2d` costmap filter that applies
  configured ROS parameter overrides to target nodes based on the mask value at the
  robot's pose.
- `zone_decision` (`diagnostic_msgs/DiagnosticArray`) — published on every zone
  transition and every enforcement failure, carrying `level`, an operator-readable
  `message`, and the `zone_state` / `mask_state` / `enforced` / `configured` /
  `unconfirmed_targets` / `degraded_targets` / `pending_parameter_sets` / `targets` /
  `event` values.
- Three-valued `enforced` (`yes` / `pending` / `NO`), so an unanswered parameter set
  is distinguishable from a confirmed one.
- A deadline on outstanding parameter sets, so a target that never answers resolves
  to `NO` rather than remaining outstanding indefinitely.

### Changed
- A rejected, failed, or unanswered parameter set is logged at `ERROR`, latches
  `enforcementDegraded()`, and leaves the costmap running, instead of propagating an
  exception out of `process()`.
- A mask value naming a state with no configuration is reported through
  `zone_decision` as `enforced: NO` rather than raising.

### Fixed
- A mask value naming an undeclared state was recorded as the current state, so the
  next transition's reset did not fire and the previous zone's limits persisted into
  a zone that had not requested them.
- `resetFilter()` cleared the degraded flag while the configuration it referred to
  remained loaded; `CostmapFilter::reset()` then re-ran the configuration load over
  it, and `nominal_defaults_` accumulated duplicate entries on each reset.
- `enforced: yes` was published on the same cycle the parameter sets were issued,
  before any confirmation had been received.
- `kMaxPendingSets` was declared but never referenced, and could not have bounded a
  single unanswered set in any case; a deadline replaces it.

### Known limitations
See **Read this before you deploy it** in `README.md`. In particular:

- ⚑ **This does not build against a released nav2.** `src/zone_parameter_filter.cpp:198`
  needs `nav2_costmap_2d::ZONE_PARAMETER_FILTER`, which is absent from tags `1.5.0` and
  `1.5.1` and present only on branches `main` and `lyrical`. Measured 2026-09-06 against
  tag `1.5.1` (`a6354f3f`) from a clean workspace: compile error. Verified to build and
  test green against upstream `lyrical` HEAD `6f23b11c` only.
  ⚑ **Superseded in [Unreleased], 2026-09-06 — see *"This package now builds against a
  RELEASED nav2"* under Fixed.** Kept here because it was true of this state when written.
- This filter cannot signal to the navigation stack that its output is untrustworthy.
- It has not been run on hardware.
