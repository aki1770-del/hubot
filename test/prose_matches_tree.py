#!/usr/bin/env python3
"""Re-derive from the tree what the prose asserts about it, and fail when they disagree.

WHY THIS EXISTS
---------------
Four figures about this package's own tests were published simultaneously and no two
agreed -- `28 of 28`, `23 of 26`, `35 tests`, `46` -- and the README cited three `throw`
line numbers that had all moved. Each was true when written. Nothing re-derived any of
them, so each went false silently at the next edit, and one of them went false twice in
a single day. The defect is not the wrong number; it is that a number was written down
once and then trusted forever.

So this is a FILE TEST, not a state-of-mind test: it reads the tree, computes the answer,
and compares. It cannot be satisfied by intending to be careful.

HONEST BOUNDS -- what this does NOT do, stated here rather than discovered later
--------------------------------------------------------------------------------
* CHK-2 vs CHK-2T is a SPLIT, and the split is the honest bound. CHK-2 compares two files
  and therefore returns the same verdict wherever the tree came from. CHK-2T needs a git
  tag, which a `git clone --depth 1` does not fetch, so it reports UNVERIFIED there rather
  than RED. UNVERIFIED IS NOT A PASS -- it means this clone could not supply the evidence.
  Judging release state from a shallow clone is outside what this script can do, and the
  previous version pretended otherwise by calling the absence a defect.
* CHK-2T reads only the HIGHEST semver tag REACHABLE FROM HEAD. Tags on other branches,
  and tags that exist only on a remote, are invisible to it.
* CHK-3 guards ONE canonical block. Environment-scoped results elsewhere in the prose
  ("23 of 26 against tag 1.5.1") are legitimately different quantities measured on
  different trees, and this script does not adjudicate them. It requires that the tree's
  own counts appear correctly in exactly one place.
* CHK-8 reads a NAMED list of directories (SHIP_DIRS) and nothing else. A new directory
  that ships files and is installed by no rule is invisible to it, and the run still prints
  GREEN -- the check is about three names, not about the tree. CHK-6 has the same shape for
  XML (it reads XML_FILES, not every `.xml`); CHK-7 does not, because it walks SHIP_DIRS for
  every `.yaml` under them.
* CHK-4 catches phrasings this project RETIRED. It is not a general drift detector: a
  brand-new doc sentence that misdescribes the code passes. The retired list grows by one
  line each time an operator sentence is reworded.
* ⚑ LINE-NUMBER CITATIONS IN PROSE ARE MOSTLY UNCHECKED, AND THIS IS THE ONE PLACE THAT
  SAYS SO. Re-derived 2026-09-06 over README, CHANGELOG, CMakeLists and doc/*.md:
  **43** `cpp:NNN`, **16** `hpp:NNN`, **6** `README.md:NNN` -- 65 in all. CHK-1 mechanically
  covers **3** of them (the `throw` sites, which are identified by their own text). The
  remaining 62 are not mechanically derivable: whether `cpp:728` still points at what its
  author meant cannot be computed, and a gate that pretended otherwise would be
  unsatisfiable. Repairs are done by hand, against a quoted anchor, and only where the
  prose quotes or names something findable in the source.
  - That earlier read "44 of 46". Both figures were true when written. **The count moves
    every time anyone edits either side**, which is the defect this whole file exists for,
    reproduced in its own docstring. Re-derive it; do not cite it.
  - The `README.md:NNN` class is the least stable of the three, because prose moves more
    often than code. All 6 were invalidated at once on 2026-09-06 when the README's first
    screen was reordered, and were repaired by hand in the same commit. Nothing detects
    the next such break.
  - Known residue after the 2026-09-06 pass: `doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md`
    mentions `cpp:479` deliberately, as the record of a citation that WAS wrong and was
    repaired to `cpp:585`. It is a historical mention, not a live citation, and it is the
    only one that intentionally points at nothing.

Usage:  prose_matches_tree.py [ROOT]      run the checks   (exit 1 on any RED)
        prose_matches_tree.py --selftest  prove each check can FAIL
"""
import ast
import io
import os
import re
import shutil
import subprocess
import sys
import tempfile
import xml.dom.minidom

SRC = "src/zone_parameter_filter.cpp"
PKG = "package.xml"
CHG = "CHANGELOG.md"
RDM = "README.md"
CML = "CMakeLists.txt"
DOCDIR = "doc"

# --- CHK-6/7/8: the BRING-UP the package ships. A launch file, its parameters and its
# maps are only real if they PARSE and if `install()` actually puts them in the install
# space -- the launch file resolves everything through
# get_package_share_directory('hubot') and can read nothing else. ---
SHIP_DIRS = ("launch", "params", "maps")
XML_FILES = (PKG, "hubot_plugins.xml")

# --- CHK-11: THE DOOR. Until 2026-09-08 this package had no CONTRIBUTING.md, no
# SECURITY.md and no issue form, and the front page told a reader whose demo run went
# wrong to "say so" without ever naming anyone to say it to. The door was then added --
# and this gate ran GREEN over all fourteen checks without reading one byte of it,
# because every check above is keyed to a NAMED file. A door is not a door if nobody can
# find it, and an unread door rots exactly like an unread number: silently, and in the
# direction of looking fine.
#
# Three limbs, because there are three separate ways a door stops working and only one
# of them is visible in a diff: the file goes missing; the page stops linking to it; or
# the form and the page stop agreeing about what a reporter has to send.
DOOR_FILES = ("CONTRIBUTING.md", "SECURITY.md")
ISSUE_TEMPLATE_DIR = os.path.join(".github", "ISSUE_TEMPLATE")

# The four facts a report needs before anyone can act on it without an interview. Each
# entry is (the form field id that must be REQUIRED, a token CONTRIBUTING.md must carry
# so a reporter can produce that fact without asking us how). Both sides are checked,
# for the reason CHK-2 checks both of its operands: a form demanding a fact the page
# never explains is an interview with extra steps, and a page naming a fact the form
# does not collect is a promise the tracker will not keep.
REPORT_FACTS = (
    ("distro", "$ROS_DISTRO"),
    ("nav2-version", "nav2_costmap_2d"),
    ("arch", "uname"),
    ("zone-decision", "zone_decision"),
)

# --- CHK-4: operator sentences this project has RETIRED. Prose still carrying one of
# these is quoting a sentence the binary no longer emits. Append on every reword. ---
# Each must be long enough to be OURS. The first cut of this list carried the four-word
# "because nobody is", which matched `doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md:31`
# ("latent because nobody is running it") -- an unrelated English sentence. A gate that
# cries wolf on ordinary prose gets switched off, so every entry here is a full clause.
RETIRED_PHRASES = [
    "is NOT being enforced on at least one target",
    "Do not rely on its limits until this reads yes",
    "Decide as if nobody is watching, because nobody is",
    "are being applied and nobody is watching",
    "what is in force is still zone",
    "NOT WATCHING. This filter has not been driven even once",
    "486 passing tests",
    "Build: never attempted",
]

# --- SC-9: tokens that turn a report into an instruction to the ROBOT's operator about
# the ROBOT. hubot may say what it knows; it may never say what to do with the machine. ---
FORBIDDEN_TOKENS = [
    "slow", "stop", "speed", "proceed", "safe", "manual",
    "take control", "seconds", "metres", "meters", "ahead",
]
IMPERATIVE_MARKERS = ["Decide as if", "Do not rely on"]


def ver(s):
    """(major, minor, patch) or None. Tolerates a leading `v`."""
    m = re.fullmatch(r"v?(\d+)\.(\d+)\.(\d+)", (s or "").strip())
    return tuple(int(g) for g in m.groups()) if m else None


def read(root, rel):
    with open(os.path.join(root, rel), encoding="utf-8") as f:
        return f.read()


def doc_files(root):
    out = [RDM, CML]
    # The door files ship publicly and are prose about this tree like any other, so
    # they are inside the prose checks rather than beside them. They are appended
    # conditionally because CHK-11 is the check that rules on their ABSENCE; a missing
    # door must produce one clear RED there, not a crash here.
    out += [n for n in DOOR_FILES if os.path.isfile(os.path.join(root, n))]
    d = os.path.join(root, DOCDIR)
    if os.path.isdir(d):
        out += [os.path.join(DOCDIR, n) for n in sorted(os.listdir(d)) if n.endswith(".md")]
    return out


# ⚑ THE REACH OF EVERY CHECK ABOVE, STATED AS A CHECK. `doc_files()` globs `.md`
# and nothing else, so every prose check in this file is blind to any other
# document format -- and a text search over a ZIP container returns 0 for every
# pattern, which reads exactly like a clean result. On 2026-09-06 that cost a
# real one: `doc/SPEC_COVERAGE.docx` was tracked, dated 2026-09-05, and its front
# page still read "THE PACKAGE AS COMMITTED WILL NOT BUILD" long after
# `doc/SPEC_COVERAGE.md:8` recorded "SUPERSEDED 2026-09-06. IT BUILDS". No
# instrument here could see it, and a pre-publish audit that searched it as text
# CLEARED it -- the conclusion happened to be wrong, and the warrant was absent
# either way.
#
# The remedy is not to teach this file every container format. It is to refuse to
# let a document exist here that this file cannot read, so the coverage claim and
# the coverage are the same size. A coverage claim that overstates its reach is
# the defect class this whole package is about.
def unreadable_docs(root):
    """(offenders, doc_dir_present). Any file under doc/ this gate cannot read."""
    d = os.path.join(root, DOCDIR)
    if not os.path.isdir(d):
        return [], False
    return sorted(n for n in os.listdir(d)
                  if os.path.isfile(os.path.join(d, n)) and not n.endswith(".md")), True


# ---------------------------------------------------------------- branch extraction
def message_branches(src):
    """Return [(level, [string literals]), ...] for publishDecision's if/else chain."""
    start = src.index("void ZoneParameterFilter::publishDecision(")
    end = src.index("\nvoid ZoneParameterFilter::resetFilter", start)
    body = src[start:end]
    chain = body[body.index("  if (enforcement_degraded_)"):body.index("  const auto add =")]
    # split on branch boundaries at 2-space indent
    parts = re.split(r"\n(?=  \}? ?else\b|  if \()", chain)
    out = []
    for p in parts:
        if "st.message" not in p:
            continue
        m = re.search(r"DiagnosticStatus::(ERROR|WARN|STALE|OK)", p)
        level = m.group(1) if m else "?"
        seg = p[p.index("st.message"):]
        # string literals, minus the ones inside // comments
        code = "\n".join(ln for ln in seg.split("\n")
                         if not ln.lstrip().startswith("//"))
        lits = re.findall(r'"((?:[^"\\]|\\.)*)"', code)
        out.append((level, lits))
    return out


# --- SC-11: the OTHER sentences a person reads. -----------------------------
# SC-9 reads publishDecision()'s branch messages and nothing else. Those are not
# the only operator-facing text this binary emits: markTargetDegraded()'s `why`
# is carried on the `event` field, and since 2026-09-06 describeUnansweredTarget()
# writes the longest and most directive sentence in the package -- the one that
# tells a person whether there is a node to walk to at all. That text was
# OUTSIDE every oracle in this file on the day it was written, which is how the
# sentence that decides a 03:00 walk came to be the least-guarded string here.
DEGRADE_FNS = [
    "std::string ZoneParameterFilter::describeUnansweredTarget(",
    "void ZoneParameterFilter::checkPendingParameterUpdates()",
    "void ZoneParameterFilter::issueAsyncSetParameters(",
]


def degrade_reason_literals(src):
    """String literals in the failure sentences that reach `event`.

    Returns (literals, functions_found). A caller must treat a short
    functions_found as a FAILURE to evaluate and never as a pass: if these
    functions are renamed, a silent empty result would report GREEN over an
    unread surface, which is the absent-verdict-reads-as-a-pass defect this
    project has already paid for once.
    """
    lits, found = [], 0
    for fn in DEGRADE_FNS:
        start = src.find(fn)
        if start < 0:
            continue
        end = src.find("\n}\n", start)
        if end < 0:
            continue
        found += 1
        body = src[start:end]
        code = "\n".join(ln for ln in body.split("\n")
                          if not ln.lstrip().startswith("//"))
        lits.extend(re.findall(r'"((?:[^"\\]|\\.)*)"', code))
    return lits, found


# ---------------------------------------------------------------- the checks
def run_checks(root):
    res = []
    src = read(root, SRC)

    # ---- CHK-1  throw line numbers cited in the README are the tree's throw lines ----
    derived = [i + 1 for i, ln in enumerate(src.split("\n"))
               if re.match(r"\s*throw ", ln)]
    rdm = read(root, RDM)
    m = re.search(r"`src/zone_parameter_filter\.cpp:(\d+)`[^\n]*\n?[^\n]*?`:(\d+)`"
                  r"[^\n]*\n?[^\n]*?`:(\d+)`", rdm)
    if not m:
        res.append(("CHK-1", False, "README no longer cites three throw sites in the "
                                    "expected form -- the check cannot be evaluated, "
                                    "which is not a pass"))
    else:
        cited = [int(g) for g in m.groups()]
        ok = cited == derived
        res.append(("CHK-1", ok, "README cites throw sites %s; tree has %s" % (cited, derived)))

    # ---- CHK-2  package.xml and the newest CHANGELOG heading name the same version ----
    # ⚑ CLONE-INDEPENDENT BY CONSTRUCTION, and that is the whole point of the split.
    # Both operands are files in the tree, so this limb returns the SAME verdict on the
    # author's host and in a fresh shallow clone of the same commit.
    #
    # It used to also require a git tag, in one conjunction, and that made the gate report
    # a different verdict about the SAME COMMIT depending on how the tree had been
    # obtained: GREEN on the author's machine, `CHK-2 RED tag=<none>` in a clean-room
    # `git clone --depth 1`, which fetches no tag objects. A works-on-my-machine defect
    # inside the instrument built to prevent works-on-my-machine defects.
    #
    # ⚑ AND IT WAS THIS PACKAGE'S OWN PI-CLASS, MIRRORED. PI-CLASS is "the reassuring
    # value is computed from the absence of negative evidence." CHK-2 computed the
    # ALARMING value from the absence of evidence: `tag=<none>` is produced both by "this
    # tree genuinely has no tag" and by "nothing fetched the tags", and the check called
    # both of them a defect. The filter itself refuses that conflation -- a silent costmap
    # reports STALE, not ERROR, because "nothing was measured and found bad; nothing was
    # measured." The gate now does to itself what the filter does to the costmap.
    pk = re.search(r"<version>([^<]+)</version>", read(root, PKG))
    pkv = pk.group(1).strip() if pk else "<none>"
    ch = re.search(r"^## \[(\d+\.\d+\.\d+)\]", read(root, CHG), re.M)
    chv = ch.group(1) if ch else "<none>"
    res.append(("CHK-2", pkv != "<none>" and pkv == chv,
                "package.xml=%s CHANGELOG=%s" % (pkv, chv)))

    # ---- CHK-2T  the newest git tag, WHEN THIS CLONE CARRIES ONE ----
    # Three outcomes, and the third is the reason this is a separate check:
    #   tag NEWER than the declared version -> RED. The tree carries a release the prose
    #       has not caught up to: prose asserting a version the tree does not carry.
    #   tag EQUAL                           -> GREEN, at a release.
    #   tag OLDER (or absent-but-present-tree) -> GREEN. An untagged commit ahead of the
    #       last tag is the NORMAL state between releases and must never be RED for that
    #       alone; the detail line names which state you are in.
    #   NO TAG VISIBLE                      -> UNVERIFIED. Not a pass and not a failure.
    #       Exit status is unaffected, because a gate that cannot be satisfied in a
    #       shallow clone gets switched off, and a switched-off gate measures nothing.
    #       It prints UNVER rather than GREEN because an absent verdict must not read
    #       like a pass.
    # ⚑ THE HIGHEST semver TAG REACHABLE FROM HEAD -- not the NEAREST one.
    # `git describe --tags --abbrev=0` answers "which tag is closest behind HEAD", and
    # when two tags sit on the same commit it may hand back the older. The question this
    # check asks is "has this tree released something the prose has not caught up to",
    # and only the highest reachable tag answers it. ⚑ The first cut of this check used
    # `describe` and the "tag a newer version" control below reported CONTROL FAILED --
    # the control found the defect in the check, which is what it is for.
    tags = subprocess.run(["git", "-C", root, "tag", "--merged", "HEAD"],
                          capture_output=True, text=True).stdout.split()
    sem = [x for x in tags if ver(x)]
    tag = max(sem, key=ver) if sem else (tags[0] if tags else "")
    if not tag:
        res.append(("CHK-2T", None,
                    "no tag object in this clone (shallow clone, or no .git) -- the tag "
                    "limb could not be evaluated. UNVERIFIED is not cleared. CHK-2 above "
                    "ran on files and is authoritative here."))
    elif ver(tag) is None or ver(pkv) is None:
        res.append(("CHK-2T", None,
                    "tag=%s package.xml=%s -- not both x.y.z, cannot compare" % (tag, pkv)))
    elif ver(tag) > ver(pkv):
        res.append(("CHK-2T", False,
                    "tag=%s is NEWER than package.xml=%s -- the prose is behind a release "
                    "this tree carries" % (tag, pkv)))
    elif ver(tag) == ver(pkv):
        # NOT "this commit is at a release": HEAD may be any number of commits past the
        # tag. That distance is exactly the quantity that is not clone-independent, so it
        # is deliberately not measured here.
        res.append(("CHK-2T", True,
                    "tag=%s == package.xml -- the prose names the newest release this "
                    "tree carries" % tag))
    else:
        res.append(("CHK-2T", True,
                    "package.xml=%s is ahead of newest tag=%s -- unreleased work between "
                    "releases, which is normal and is not a defect" % (pkv, tag)))

    # ---- CHK-3  the tree's own test counts appear in the canonical block ----
    tdir = os.path.join(root, "test")
    cases = 0
    for n in sorted(os.listdir(tdir)):
        if n.endswith(".cpp"):
            cases += len(re.findall(r"^TEST(?:_F)?\(", read(root, "test/" + n), re.M))
    cml = read(root, CML)
    targets = len(re.findall(r"ament_add_gtest\(", cml)) + len(re.findall(r"add_test\(NAME", cml))
    spec = read(root, os.path.join(DOCDIR, "SPEC_COVERAGE.md"))
    want = ("DERIVED FROM THE TREE: %d gtest cases in `test/*.cpp`, %d CTest targets."
            % (cases, targets))
    res.append(("CHK-3", want in spec,
                "expected canonical line in doc/SPEC_COVERAGE.md: %r" % want))

    # ---- CHK-4  no retired operator sentence survives in the prose ----
    # \u26d1 QUOTED TEXT IS EXEMPT, and it has to be: this project corrects a false
    # sentence by KEEPING it and marking it, so a reader learns what they were told
    # before. The correction quotes the retired words, and a gate that could not tell a
    # quotation from a claim would forbid the honest form of the fix. An occurrence is
    # inside a quotation when an odd number of `"` precede it in the file, which handles
    # the multi-line block quotes this repo actually uses.
    bad = []
    for rel in doc_files(root):
        txt = read(root, rel)
        for ph in RETIRED_PHRASES:
            at = txt.find(ph)
            while at != -1:
                if txt.count('"', 0, at) % 2 == 0:      # not inside a quotation
                    bad.append("%s: %r" % (rel, ph))
                    break
                at = txt.find(ph, at + 1)
    res.append(("CHK-4", not bad, "retired phrasing still published -> " + "; ".join(bad)
                if bad else "no retired phrasing in prose"))

    # ---- SC-9  no message branch tells a person what to do with the robot ----
    branches = message_branches(src)
    hits = []
    for level, lits in branches:
        blob = " ".join(lits).lower()
        for tok in FORBIDDEN_TOKENS:
            if re.search(r"\b" + re.escape(tok), blob):
                hits.append("%s branch contains %r" % (level, tok))
    res.append(("SC-9", not hits,
                "forbidden token in operator sentence -> " + "; ".join(hits)
                if hits else "%d branches, no forbidden token" % len(branches)))

    # ---- SC-10  degraded branches instruct belief; reassuring branches instruct nothing ----
    prob = []
    degraded = reassuring = 0
    for level, lits in branches:
        blob = " ".join(lits)
        has = any(mk in blob for mk in IMPERATIVE_MARKERS)
        if level == "OK":
            reassuring += 1
            if has:
                prob.append("OK branch carries an imperative: %r" % blob[:70])
        else:
            degraded += 1
            if not has:
                prob.append("%s branch carries NO imperative: %r" % (level, blob[:70]))
    if degraded != 4 or reassuring != 2:
        prob.append("branch shape moved: %d degraded / %d reassuring (expected 4 / 2)"
                    % (degraded, reassuring))
    res.append(("SC-10", not prob,
                "; ".join(prob) if prob else
                "%d degraded all instruct belief; %d reassuring instruct nothing"
                % (degraded, reassuring)))

    # ---- CHK-5  no document exists here that this gate cannot read ----
    offenders, present = unreadable_docs(root)
    if not present:
        res.append(("CHK-5", False,
                    "doc/ is absent -- the check cannot be evaluated, which is not a pass"))
    else:
        res.append(("CHK-5", not offenders,
                    "doc/ holds a file no check in this gate can read -> " +
                    "; ".join(offenders) +
                    " (a text search over a container returns 0 for every pattern, "
                    "which reads exactly like a clean result)"
                    if offenders else
                    "every file in doc/ is readable prose this gate actually checks"))

    # ---- SC-11  the failure sentences carried on `event` obey SC-9's rule too ----
    lits, found = degrade_reason_literals(src)
    if found < len(DEGRADE_FNS) or not lits:
        res.append(("SC-11", False,
                    "only %d of %d failure-sentence functions were found (%d literals) -- "
                    "the check cannot be evaluated, which is not a pass"
                    % (found, len(DEGRADE_FNS), len(lits))))
    else:
        blob = " ".join(lits).lower()
        hits = [tok for tok in FORBIDDEN_TOKENS
                if re.search(r"\b" + re.escape(tok), blob)]
        res.append(("SC-11", not hits,
                    "forbidden token in a failure sentence -> " + "; ".join(hits)
                    if hits else
                    "%d literals across %d failure-sentence functions, no forbidden token"
                    % (len(lits), found)))

    # ---- CHK-6  every XML and Python file this package ships actually parses ----
    # ⚑ THIS CHECK EXISTS BECAUSE A WARNING DID NOT WORK. package.xml carries a comment
    # saying, in its own words, that XML forbids a double hyphen inside a comment and
    # that an earlier draft had "used the flag's real spelling and made package.xml
    # unparseable". On 2026-09-06 the very next person to edit that file broke it the
    # same way THREE TIMES in one sitting, twice within twenty lines of the warning.
    # A rule that is written, read and even quoted still does not fire; only a check
    # the author cannot decline does. Stdlib only -- no new dependency, so this runs on
    # the ROS-free host the toolchain-free lane was written for.
    broke = []
    for rel in XML_FILES:
        fp = os.path.join(root, rel)
        if not os.path.isfile(fp):
            broke.append("%s: MISSING" % rel)
            continue
        try:
            xml.dom.minidom.parse(fp)
        except Exception as e:            # noqa: BLE001 - any parse failure is the finding
            broke.append("%s: %s" % (rel, e))
    pys = []
    for d in SHIP_DIRS:
        dd = os.path.join(root, d)
        if os.path.isdir(dd):
            pys += [os.path.join(d, n) for n in sorted(os.listdir(dd)) if n.endswith(".py")]
    for rel in pys:
        try:
            ast.parse(read(root, rel))
        except Exception as e:            # noqa: BLE001
            broke.append("%s: %s" % (rel, e))
    res.append(("CHK-6", not broke,
                "a shipped XML/Python file does not parse -> " + "; ".join(broke)
                if broke else
                "%d XML + %d Python files parse" % (len(XML_FILES), len(pys))))

    # ---- CHK-7  every shipped YAML parses (UNVERIFIED where pyyaml is absent) ----
    # Tri-state on purpose, the same way CHK-2T is: pyyaml is NOT stdlib, and this file
    # promises to run "anywhere, with nothing installed". Reporting GREEN on a host that
    # could not open a single one of these files would be the absent-verdict-reads-as-a-
    # pass defect, inside the gate built to catch it.
    yamls = []
    for d in SHIP_DIRS:
        for dirpath, _dirs, names in os.walk(os.path.join(root, d)):
            yamls += [os.path.join(dirpath, n) for n in sorted(names) if n.endswith(".yaml")]
    try:
        import yaml as _yaml
    except ImportError:
        res.append(("CHK-7", None,
                    "pyyaml is not installed here, so the %d shipped .yaml files were "
                    "NOT parsed. UNVERIFIED is not cleared." % len(yamls)))
    else:
        ybad = []
        for fp in yamls:
            try:
                _yaml.safe_load(open(fp, encoding="utf-8"))
            except Exception as e:        # noqa: BLE001
                ybad.append("%s: %s" % (os.path.relpath(fp, root), e))
        res.append(("CHK-7", not ybad,
                    "a shipped .yaml does not parse -> " + "; ".join(ybad) if ybad else
                    "%d shipped .yaml files parse" % len(yamls)))

    # ---- CHK-8  what the tree ships, install() actually installs ----
    # ⚑ THE DIFFERENCE BETWEEN COMMITTED AND SHIPPED, AS A CHECK. A launch file, its
    # params and its maps that exist in git but are named by no install(DIRECTORY ...)
    # rule are invisible to `ros2 launch`: the integrator gets "file not found" for a
    # file she can plainly see in the repository. That failure costs a container run to
    # discover and costs nothing to catch here.
    cml = read(root, CML)
    installed_dirs = set()
    for m in re.finditer(r"install\s*\(\s*DIRECTORY([^)]*)\)", cml):
        installed_dirs.update(re.findall(r"[A-Za-z0-9_./]+", m.group(1)))
    missing = [d for d in SHIP_DIRS
               if os.path.isdir(os.path.join(root, d)) and d not in installed_dirs]
    # ⚑ THE DETAIL LINE NAMES WHAT WAS CHECKED, NOT "everything". CHK-8 reads the
    # NAMED list SHIP_DIRS and is blind to any other directory: add a `config/` and
    # this check says nothing about it while still printing GREEN. Stated here and in
    # the docstring because a coverage claim that overstates its reach is the defect
    # class this whole package is about -- CHK-5 exists for the same reason one file
    # over. The first draft of this line read "every shipped directory that exists",
    # which is a claim about the tree; it is a claim about three names.
    checked = [d for d in SHIP_DIRS if os.path.isdir(os.path.join(root, d))]
    res.append(("CHK-8", not missing,
                "directory exists in the tree and NO install(DIRECTORY) rule ships it, so "
                "ros2 launch cannot read it -> " + "; ".join(missing) if missing else
                "%d of the %d NAMED dirs exist and each is named by an install() rule "
                "(%s); directories outside that list are not examined"
                % (len(checked), len(SHIP_DIRS), ", ".join(checked))))

    # ---- CHK-9  the workflow that runs every check above still declares its triggers ----
    # ⚑ WHY THIS CHECK EXISTS AND EXACTLY WHAT IT CANNOT DO.
    # Every other check in this file is only ever run because .github/workflows/gate.yml
    # tells GitHub to run it. That file is therefore the one document in this repository
    # whose failure is SILENT: a workflow with no `pull_request:` trigger does not report
    # red, it reports NOTHING, and a pull request with no checks looks exactly like a pull
    # request with nothing to check. That is the absent-verdict-reads-as-a-pass shape,
    # sitting one layer above every check built to catch it.
    #
    # ⚑ THE BOUND, STATED RATHER THAN GLOSSED. This check CANNOT save a pull request that
    # removes its own trigger: GitHub decides whether to run a workflow by reading the
    # workflow file as that pull request would leave it, so a PR deleting `pull_request:`
    # prevents the very run that would have caught it. Closing THAT requires a REQUIRED
    # STATUS CHECK in branch protection -- a repository setting, not a file -- and no test
    # in this tree can substitute for it. What this check does catch is the ordinary case:
    # an edit that breaks the workflow's YAML or drops a trigger, caught on some later run
    # rather than never.
    wf = os.path.join(".github", "workflows", "gate.yml")
    wfp = os.path.join(root, wf)
    if not os.path.isfile(wfp):
        res.append(("CHK-9", False,
                    "%s is absent -- nothing runs the checks above automatically" % wf))
    else:
        try:
            import yaml as _y9
        except ImportError:
            res.append(("CHK-9", None,
                        "pyyaml absent, so the workflow could not be parsed. UNVERIFIED is "
                        "not cleared: this limb says nothing about whether CI still runs."))
        else:
            try:
                doc = _y9.safe_load(read(root, wf))
            except Exception as e:
                res.append(("CHK-9", False,
                            "%s does not parse as YAML, so GitHub runs NOTHING from it -> %s"
                            % (wf, e)))
            else:
                # `on:` is YAML 1.1 true; safe_load gives the boolean key, not the string.
                trig = doc.get("on", doc.get(True)) if isinstance(doc, dict) else None
                names = set(trig) if isinstance(trig, dict) else (
                    set(trig) if isinstance(trig, list) else {trig})
                want = {"push", "pull_request"}
                miss = sorted(want - names)
                res.append(("CHK-9", not miss,
                            ("%s no longer declares %s -- those events run NO checks and "
                             "report NOTHING" % (wf, ", ".join(miss))) if miss else
                            "%s declares %s; a push and a pull request each run the gate"
                            % (wf, ", ".join(sorted(want)))))

    # ---- CHK-10  no job hand-writes a ROS package list ----
    # ⚑ THIS CHECK EXISTS BECAUSE THE DEFECT IT GUARDS ALREADY HAPPENED, ON THE FIRST CI
    # RUN THIS PACKAGE EVER HAD. The workflow's two build jobs each carried their own
    # hand-written apt list, and the `suite` copy was produced by deleting lines from the
    # `launch` copy. The deletion took `nav2_map_server` with it, and with it the
    # transitively-installed `diagnostic_msgs` -- which package.xml:55 declares and
    # CMakeLists.txt:24 requires. `launch` went green and `suite` died in 1.55 s at
    # find_package. Two copies of one fact, and only one of them was ever built against.
    #
    # The dependencies now come from package.xml via rosdep, inside ONE composite action
    # both jobs call. This check keeps it that way: a `ros-lyrical-*` written into the
    # workflow is a second list being born.
    #
    # ⚑ THE FIRST DRAFT OF THIS CHECK SPLIT THE WORKFLOW TEXT ON "\n  " TO FIND A JOB'S
    # BODY -- which is a prefix of every 4-space-indented line, so each body truncated to
    # nothing and the check reported RED on a tree that was correct. It is parsed now,
    # not pattern-matched. A check that cries wolf is retired by the people it interrupts.
    wf10 = os.path.join(".github", "workflows", "gate.yml")
    actrel = "./.github/actions/ros-substrate"
    actp = os.path.join(root, ".github", "actions", "ros-substrate", "action.yml")
    try:
        import yaml as _y10
    except ImportError:
        res.append(("CHK-10", None,
                    "pyyaml absent, so the workflow could not be parsed. UNVERIFIED is not "
                    "cleared: this says nothing about whether a second package list exists."))
    else:
        problems = []
        if not os.path.isfile(actp):
            problems.append("the shared substrate action is missing, so each job must be "
                            "assembling its own")
        if not os.path.isfile(os.path.join(root, wf10)):
            problems.append("%s is absent" % wf10)
        else:
            wtxt = read(root, wf10)
            strays = sorted(set(re.findall(r"ros-lyrical-[a-z0-9-]+", wtxt)))
            if strays:
                problems.append("the workflow names ROS packages directly (%s) instead of "
                                "leaving them to package.xml + rosdep" % ", ".join(strays))
            try:
                wdoc = _y10.safe_load(wtxt) or {}
            except Exception as e:
                problems.append("%s does not parse (%s)" % (wf10, e))
                wdoc = {}
            for job, spec in (wdoc.get("jobs") or {}).items():
                steps = (spec or {}).get("steps") or []
                builds = any("colcon build" in (st.get("run") or "") for st in steps)
                if builds and not any(st.get("uses") == actrel for st in steps):
                    problems.append("job '%s' builds but does not use the shared substrate "
                                    "action" % job)
        res.append(("CHK-10", not problems,
                    "; ".join(problems) if problems else
                    "every building job takes its dependencies from package.xml through the "
                    "one shared substrate action; no ROS package is named in the workflow"))

    # ---- CHK-11  the door exists, the page links it, and the form and the page agree ----
    problems = []
    missing = [n for n in DOOR_FILES if not os.path.isfile(os.path.join(root, n))]
    if missing:
        problems.append("absent: " + ", ".join(missing))

    # A door nobody can find is not a door. The link is checked in the README because
    # that is the page a reader arrives on; a file present in the tree and named nowhere
    # is reachable only by someone already browsing the repository root.
    rdm = read(root, RDM)
    unlinked = [n for n in DOOR_FILES
                if os.path.isfile(os.path.join(root, n)) and ("(%s)" % n) not in rdm]
    if unlinked:
        problems.append("in the tree but not linked from README.md: " + ", ".join(unlinked))

    forms, unparsed = {}, []
    d = os.path.join(root, ISSUE_TEMPLATE_DIR)
    if not os.path.isdir(d):
        problems.append("no %s -- a reporter has to guess what to send" % ISSUE_TEMPLATE_DIR)
    else:
        import yaml
        for n in sorted(os.listdir(d)):
            if not n.endswith((".yml", ".yaml")):
                continue
            try:
                forms[n] = yaml.safe_load(read(root, os.path.join(ISSUE_TEMPLATE_DIR, n)))
            except Exception as e:
                unparsed.append("%s (%s)" % (n, type(e).__name__))
        if unparsed:
            # GitHub renders a form it cannot parse as no form at all, and says so
            # nowhere the author will see.
            problems.append("does not parse: " + ", ".join(unparsed))

    required_ids = set()
    for doc in forms.values():
        for field in (doc or {}).get("body", []) or []:
            if isinstance(field, dict) and (field.get("validations") or {}).get("required"):
                required_ids.add(field.get("id"))
    contributing = (read(root, "CONTRIBUTING.md")
                    if os.path.isfile(os.path.join(root, "CONTRIBUTING.md")) else "")
    for fid, token in REPORT_FACTS:
        if fid not in required_ids:
            problems.append("no required form field `%s`" % fid)
        if contributing and token not in contributing:
            problems.append("CONTRIBUTING.md does not say how to get `%s`" % token)

    res.append(("CHK-11", not problems,
                "; ".join(problems) if problems else
                "%s present and linked from README; %d issue form(s) parse; the %d facts "
                "CONTRIBUTING explains are the %d the form requires"
                % (" + ".join(DOOR_FILES), len(forms), len(REPORT_FACTS), len(REPORT_FACTS))))
    return res


# ---------------------------------------------------------------- negative controls
def _chk1_mutate(text):
    """Break exactly the number CHK-1 reads, found with CHK-1's own pattern.

    ⚑ THIS WAS A HARD-CODED `:105` AND IT WENT STALE THE DAY THE CODE MOVED.
    The jazzy/kilted port moved the throw sites (105/270/339 -> 118/314/397). CHK-1's
    README citations were repaired with it; THIS CONTROL'S LITERAL WAS NOT. So the
    mutation replaced a string that no longer existed, the tree came back unchanged,
    CHK-1 stayed green -- and the control reported STILL GREEN, i.e. it had stopped
    proving anything while CHK-1 itself still passed. A check that cannot be made to
    fail has measured nothing, and this one announced that about itself only because
    the controls are run on every push. Measured 2026-09-07 on the runner; the local
    check run had been green because running a check is not running its control.

    Derived now, never literal: whatever three numbers CHK-1 reads, the first of them
    is what gets broken. It cannot go stale again when the code moves.
    """
    pat = re.compile(r"(`src/zone_parameter_filter\.cpp:)(\d+)(`[^\n]*\n?[^\n]*?`:)"
                     r"(\d+)(`[^\n]*\n?[^\n]*?`:)(\d+)(`)")
    m = pat.search(text)
    if not m:
        return text          # CHK-1 itself reports the form is gone; that is not a pass
    return text[:m.start(2)] + "999" + text[m.end(2):]


MUTATIONS = [
    ("CHK-1", RDM, _chk1_mutate),
    ("CHK-2", PKG, lambda t: t.replace("<version>", "<version>9.", 1)),
    # ⚑ THE OTHER OPERAND. CHK-2 is a two-file agreement, so it needs a control on each
    # side; mutating only package.xml would leave a check that could be satisfied by a
    # CHANGELOG nobody ever re-derived.
    ("CHK-2", CHG, lambda t: re.sub(r"^## \[\d+\.\d+\.\d+\]", "## [9.9.9]", t, 1, re.M)),
    ("CHK-3", os.path.join(DOCDIR, "SPEC_COVERAGE.md"),
     lambda t: re.sub(r"DERIVED FROM THE TREE: \d+", "DERIVED FROM THE TREE: 999", t, 1)),
    ("CHK-4", RDM, lambda t: t + "\n\nThe message reads: Zone 2 is NOT being enforced "
                                 "on at least one target.\n"),
    # \u26d1 MUTATE THE STATEMENT, NOT THE STRING. The first cut of this control
    # replaced the bare literal `"Outside any zone; nominal defaults are in force."`
    # with `replace(..., 1)` -- and the FIRST occurrence of that literal in the file is
    # inside a COMMENT at cpp:648, so the mutation landed somewhere SC-9 does not read
    # and the control reported STILL GREEN. It was right to: an oracle nobody has
    # watched go red is not evidence that it can. Anchoring on `st.message =` picks the
    # one occurrence that is code.
    ("SC-9", SRC, lambda t: t.replace(
        'st.message = "Outside any zone; nominal defaults are in force.";',
        'st.message = "Outside any zone. Slow down.";', 1)),
    ("SC-10", SRC, lambda t: t.replace('st.message = "Outside any zone; nominal defaults are in force.";',
                                       'st.message = "Outside any zone. Decide as if you may proceed.";', 1)),
    # ⚑ Anchored on a literal that exists ONLY inside describeUnansweredTarget(),
    # so a control that lands anywhere else cannot report a false pass -- the
    # lesson SC-9's own first control taught this file at cpp:648.
    ("SC-11", SRC, lambda t: t.replace(
        '"is the place to look.";',
        '"is the place to look. Stop the robot.";', 1)),
    # ⚑ THE CONTROL IS THE DEFECT ITSELF. This inserts a double hyphen inside an XML
    # comment in package.xml -- the exact break that happened three times on 2026-09-06.
    # If this mutation does not turn CHK-6 red, CHK-6 would not have caught it either.
    ("CHK-6", PKG, lambda t: t.replace("maps/. These are exec_depend",
                                       "maps/. " + "-" * 2 + " These are exec_depend", 1)),
    ("CHK-7", "params/zone_filter_demo.yaml",
     lambda t: t + "\nthis: is: not: valid: yaml:\n  - [unclosed\n"),
    # Removes `maps` from the install rule: the files stay in git and vanish from the
    # install space, which is the failure a reader cannot see by looking at the tree.
    ("CHK-8", CML, lambda t: t.replace("install(DIRECTORY launch params maps DESTINATION",
                                       "install(DIRECTORY launch params DESTINATION", 1)),
    # ⚑ THE MUTATION IS THE SILENT DEFECT ITSELF: drop `pull_request:` from the triggers.
    # A workflow mutated this way is still perfectly valid YAML and still runs on push,
    # so nothing anywhere goes red -- pull requests simply stop being checked. If this
    # control does not turn CHK-9 red, CHK-9 would not have noticed the real thing either.
    ("CHK-9", os.path.join(".github", "workflows", "gate.yml"),
     lambda t: t.replace("\n  pull_request:\n", "\n", 1)),
    # ⚑ THE MUTATION IS THE DEFECT VERBATIM: a ROS package written into a job by hand.
    # That is how the second list was born the first time, and it stayed green in one job
    # while killing the other. If this does not turn CHK-10 red, CHK-10 would not have
    # seen the real one either.
    ("CHK-10", os.path.join(".github", "workflows", "gate.yml"),
     lambda t: t.replace("ca-certificates curl gnupg git",
                         "ca-certificates curl gnupg git ros-lyrical-nav2-costmap-2d", 1)),
    # ⛑ ANCHORED ON `render: text`, WHICH OCCURS ONCE AND ONLY ON THE TRANSCRIPT
    # FIELD. Mutating the FIRST `required: true` in the file would land on
    # `what-happened`, which is not one of the four facts, and the control would report
    # STILL GREEN while CHK-11 was working perfectly -- the cpp:648 lesson SC-9's own
    # first control taught this file, in a different file.
    ("CHK-11", os.path.join(".github", "ISSUE_TEMPLATE", "bug_report.yml"),
     lambda t: t.replace("      render: text\n    validations:\n      required: true",
                         "      render: text\n    validations:\n      required: false", 1)),
    # ⛑ THE OTHER OPERAND, for the reason CHK-2 has two: the agreement can be broken
    # from the page's side as easily as from the form's, and only one of those two
    # directions is visible when you are editing the form.
    ("CHK-11", "CONTRIBUTING.md", lambda t: t.replace("$ROS_DISTRO", "the distribution")),
    # ⛑ THE LINK, NOT THE FILE. This deletes every link to CONTRIBUTING.md from the
    # README and leaves the file itself in the tree -- the exact shape of a door that
    # exists and cannot be found, which no existence check can see and which a diff of
    # the door file itself shows as no change at all. `replace` with no count, because
    # the page links it more than once and removing one would leave the check green.
    ("CHK-11", RDM, lambda t: t.replace("(CONTRIBUTING.md)", "(the contributing guide)")),
]


# ⚑ CONTROLS THAT ACT ON THE REPOSITORY, NOT ON A FILE. CHK-2T's evidence is a git tag,
# so its controls must be able to ADD one and to TAKE THEM ALL AWAY.
#
# The second is the one that had to exist. Every control above proves a check can go RED.
# None of them can see the defect that was actually here: a check that went RED on an
# absence. Proving "absence -> UNVERIFIED, and the run still exits 0" needs a control that
# expects something OTHER than red, which is why `expect` is a tri-state and not a flag.
REPO_MUTATIONS = [
    ("CHK-2T", False, "tag a version NEWER than package.xml",
     lambda dst, pkv: subprocess.run(["git", "-C", dst, "tag", bump(pkv)],
                                     capture_output=True)),
    # ⚑ The control has to CREATE the offending shape, because the defect is a file
    # that EXISTS, not a string that changed. No file-text mutation can express it,
    # which is precisely why nothing caught the real one.
    ("CHK-5", False, "add a binary doc no check can read",
     lambda dst, pkv: io.open(os.path.join(dst, DOCDIR, "regression_probe.docx"),
                              "wb").write(b"PK\x03\x04 not readable prose")),
    # ⛑ The existence limb needs a control that REMOVES A FILE, and no text mutation
    # can express that -- the same reason CHK-5's control has to create a file rather
    # than edit one.
    ("CHK-11", False, "delete SECURITY.md (the door file is gone)",
     lambda dst, pkv: os.remove(os.path.join(dst, "SECURITY.md"))),
    ("CHK-2T", None, "delete every tag (the clean-room condition)",
     lambda dst, pkv: [subprocess.run(["git", "-C", dst, "tag", "-d", tg],
                                      capture_output=True)
                       for tg in subprocess.run(["git", "-C", dst, "tag", "-l"],
                                                capture_output=True,
                                                text=True).stdout.split()]),
]


def bump(v):
    p = ver(v)
    return "%d.%d.%d" % (p[0] + 1, 0, 0) if p else "99.0.0"


def pkg_version(root):
    m = re.search(r"<version>([^<]+)</version>", read(root, PKG))
    return m.group(1).strip() if m else "0.0.0"


def _temp_repo(root, tag=True):
    """A throwaway git repo holding a copy of `root`, tagged at its declared version."""
    tmp = tempfile.mkdtemp(prefix="prose-nc-")
    dst = os.path.join(tmp, "t")
    shutil.copytree(root, dst, ignore=shutil.ignore_patterns(".git"))
    subprocess.run(["git", "-C", dst, "init", "-q"], capture_output=True)
    subprocess.run(["git", "-C", dst, "add", "-A"], capture_output=True)
    subprocess.run(["git", "-C", dst, "-c", "user.email=n@n", "-c", "user.name=n",
                    "commit", "-qm", "s"], capture_output=True)
    if tag:
        subprocess.run(["git", "-C", dst, "tag", base_tag(root)], capture_output=True)
    return tmp, dst


def selftest(root):
    print("NEGATIVE CONTROLS -- each mutation must turn exactly its own check RED.\n")
    base = {cid: ok for cid, ok, _ in run_checks(root)}
    if any(v is False for v in base.values()):
        print("  refusing to run: the unmutated tree is RED -> %s"
              % [c for c, v in base.items() if v is False])
        return 1
    print("  baseline: %d checks, none RED\n" % len(base))
    bad = 0
    for cid, rel, mut in MUTATIONS:
        tmp, dst = _temp_repo(root)
        fp = os.path.join(dst, rel)
        with open(fp, encoding="utf-8") as f:
            txt = f.read()
        with open(fp, "w", encoding="utf-8") as f:
            f.write(mut(txt))
        got = {c: ok for c, ok, _ in run_checks(dst)}
        fired = got.get(cid) is False
        print("  %-6s mutate %-28s -> %s" % (cid, rel, "RED (control PASSES)" if fired
                                             else "STILL GREEN -- CONTROL FAILED"))
        if not fired:
            bad += 1
        shutil.rmtree(tmp, ignore_errors=True)

    # ⚑ AND NOW THE OTHER DIRECTION. Above, every control removes truth and expects RED.
    # These two change the EVIDENCE and expect a named outcome -- one of them not RED.
    print()
    for cid, expect, label, mut in REPO_MUTATIONS:
        tmp, dst = _temp_repo(root)
        mut(dst, pkg_version(dst))
        got = {c: ok for c, ok, _ in run_checks(dst)}
        actual = got.get(cid, "absent")
        okc = actual is expect
        word = {False: "RED", True: "GREEN", None: "UNVER"}
        note = ""
        if expect is None:
            # The absence control carries a SECOND assertion, and it is the load-bearing
            # one: the file limb must be unaffected and the run must still exit 0. A tag
            # that was never fetched is not a defect in the package.
            hard = got.get("CHK-2") is True and not any(v is False for v in got.values())
            okc = okc and hard
            note = "  [CHK-2 still GREEN and 0 RED: %s]" % ("yes" if hard else "NO")
        print("  %-6s %-42s -> expected %-5s got %-5s %s%s"
              % (cid, label, word.get(expect, "?"),
                 word.get(actual, str(actual)),
                 "(control PASSES)" if okc else "CONTROL FAILED", note))
        if not okc:
            bad += 1
        shutil.rmtree(tmp, ignore_errors=True)

    total = len(MUTATIONS) + len(REPO_MUTATIONS)
    print("\n  %d/%d controls proved their check behaves as claimed." % (total - bad, total))
    return 1 if bad else 0


def base_tag(root):
    return subprocess.run(["git", "-C", root, "describe", "--tags", "--abbrev=0"],
                          capture_output=True, text=True).stdout.strip() or "0.0.0"


def main():
    args = [a for a in sys.argv[1:]]
    st = "--selftest" in args
    args = [a for a in args if not a.startswith("--")]
    root = os.path.abspath(args[0]) if args else os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))
    if st:
        return selftest(root)
    res = run_checks(root)
    red = unver = 0
    for cid, ok, detail in res:
        # ⚑ THREE STATES, and the third is deliberate. `None` means the evidence this
        # check needs is not present in this tree -- not that the package is fine. It
        # prints UNVER, never GREEN, because an absent verdict reads exactly like a pass.
        word = "GREEN" if ok is True else ("RED" if ok is False else "UNVER")
        print("  %-6s %-5s %s" % (cid, word, detail))
        if ok is False:
            red += 1
        elif ok is None:
            unver += 1
    print("\n  %d checks, %d RED, %d UNVERIFIED" % (len(res), red, unver))
    if unver:
        print("  UNVERIFIED is not a pass. It means this clone could not supply the "
              "evidence, and it does not affect exit status on purpose.")
    return 1 if red else 0


if __name__ == "__main__":
    sys.exit(main())
