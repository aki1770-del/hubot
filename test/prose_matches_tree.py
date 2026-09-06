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
* CHK-3 guards ONE canonical block. Environment-scoped results elsewhere in the prose
  ("23 of 26 against tag 1.5.1") are legitimately different quantities measured on
  different trees, and this script does not adjudicate them. It requires that the tree's
  own counts appear correctly in exactly one place.
* CHK-4 catches phrasings this project RETIRED. It is not a general drift detector: a
  brand-new doc sentence that misdescribes the code passes. The retired list grows by one
  line each time an operator sentence is reworded.
* 44 of this package's 46 `src/...cpp:NNN` citations in prose are NOT checked. Whether
  `cpp:728` still points at what its author meant is not mechanically derivable, and a
  gate that pretended otherwise would be unsatisfiable. CHK-1 covers the subset that IS
  derivable -- the `throw` sites -- because those are identified by their own text.

Usage:  prose_matches_tree.py [ROOT]      run the checks   (exit 1 on any RED)
        prose_matches_tree.py --selftest  prove each check can FAIL
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile

SRC = "src/zone_parameter_filter.cpp"
PKG = "package.xml"
CHG = "CHANGELOG.md"
RDM = "README.md"
CML = "CMakeLists.txt"
DOCDIR = "doc"

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


def read(root, rel):
    with open(os.path.join(root, rel), encoding="utf-8") as f:
        return f.read()


def doc_files(root):
    out = [RDM, CML]
    d = os.path.join(root, DOCDIR)
    if os.path.isdir(d):
        out += [os.path.join(DOCDIR, n) for n in sorted(os.listdir(d)) if n.endswith(".md")]
    return out


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

    # ---- CHK-2  tag / package.xml / newest CHANGELOG heading name the same version ----
    tag = subprocess.run(["git", "-C", root, "describe", "--tags", "--abbrev=0"],
                         capture_output=True, text=True).stdout.strip()
    pk = re.search(r"<version>([^<]+)</version>", read(root, PKG))
    pkv = pk.group(1).strip() if pk else "<none>"
    ch = re.search(r"^## \[(\d+\.\d+\.\d+)\]", read(root, CHG), re.M)
    chv = ch.group(1) if ch else "<none>"
    ok = tag and tag == pkv == chv
    res.append(("CHK-2", bool(ok),
                "tag=%s package.xml=%s CHANGELOG=%s" % (tag or "<none>", pkv, chv)))

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
    return res


# ---------------------------------------------------------------- negative controls
MUTATIONS = [
    ("CHK-1", RDM, lambda t: t.replace("`src/zone_parameter_filter.cpp:105`",
                                       "`src/zone_parameter_filter.cpp:999`", 1)),
    ("CHK-2", PKG, lambda t: t.replace("<version>", "<version>9.", 1)),
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
]


def selftest(root):
    print("NEGATIVE CONTROLS -- each mutation must turn exactly its own check RED.\n")
    base = {cid: ok for cid, ok, _ in run_checks(root)}
    if not all(base.values()):
        print("  refusing to run: the unmutated tree is not green -> %s"
              % [c for c, v in base.items() if not v])
        return 1
    print("  baseline: all %d checks GREEN\n" % len(base))
    bad = 0
    for cid, rel, mut in MUTATIONS:
        tmp = tempfile.mkdtemp(prefix="prose-nc-")
        dst = os.path.join(tmp, "t")
        shutil.copytree(root, dst, ignore=shutil.ignore_patterns(".git"))
        subprocess.run(["git", "-C", dst, "init", "-q"], capture_output=True)
        subprocess.run(["git", "-C", dst, "add", "-A"], capture_output=True)
        subprocess.run(["git", "-C", dst, "-c", "user.email=n@n", "-c", "user.name=n",
                        "commit", "-qm", "s"], capture_output=True)
        subprocess.run(["git", "-C", dst, "tag", base_tag(root)], capture_output=True)
        p = os.path.join(dst, rel)
        with open(p, encoding="utf-8") as f:
            t = f.read()
        with open(p, "w", encoding="utf-8") as f:
            f.write(mut(t))
        got = {c: ok for c, ok, _ in run_checks(dst)}
        fired = got.get(cid) is False
        print("  %-6s mutate %-28s -> %s" % (cid, rel, "RED (control PASSES)" if fired
                                             else "STILL GREEN -- CONTROL FAILED"))
        if not fired:
            bad += 1
        shutil.rmtree(tmp, ignore_errors=True)
    print("\n  %d/%d controls proved their check can fail." % (len(MUTATIONS) - bad, len(MUTATIONS)))
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
    red = 0
    for cid, ok, detail in res:
        print("  %-6s %-5s %s" % (cid, "GREEN" if ok else "RED", detail))
        if not ok:
            red += 1
    print("\n  %d checks, %d RED" % (len(res), red))
    return 1 if red else 0


if __name__ == "__main__":
    sys.exit(main())
