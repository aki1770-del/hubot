#!/usr/bin/env python3
"""The distro floor, executed instead of asserted.

⚑ WHY THIS FILE EXISTS.

Until 2026-09-07 the answer to "which ROS 2 distribution does hubot need?" was a
paragraph in README.md. A paragraph is the worst tier of poka-yoke: it is found
downstream, by the reader, after she has cloned. It can also go false in two
directions and neither one reddens a light:

  * generously — the claim widens, or a port half-lands, and the page promises a
    build that does not happen;
  * meanly — the port DOES land and the page still tells a jazzy developer she is
    not welcome. She leaves. Nothing is red, nothing is logged, and the cost is
    paid entirely by someone we never hear from.

The second is the expensive one and it is invisible to every gate that only
tests the supported path. So the floor is DECLARED once in
`.github/supported_distros.yml` and this file executes it:

  --matrix   emit the CI matrix from the declaration (CI does not hold its own copy)
  --probe    run in a container: prove the floor for one distribution, both directions
  --render   render the README's floor table from the declaration
  --check    fail if README's rendered block has drifted from the declaration

`--probe` does not ask "did something fail". Anything can fail; a gate that
accepts any failure has measured nothing. It probes EVERY `find_package()` in
CMakeLists.txt individually and requires the set that fails to be EXACTLY the
set the declaration names. That is what turns "jazzy is unsupported" into
something a porter can act on: it names which door is locked.
"""
import argparse, json, os, re, subprocess, sys, tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DECL = os.path.join(ROOT, ".github", "supported_distros.yml")
RDM = os.path.join(ROOT, "README.md")
BEGIN = "<!-- BEGIN GENERATED distro-floor — edit .github/supported_distros.yml, then run scripts/distro_floor.py --render -->"
END = "<!-- END GENERATED distro-floor -->"


def declaration():
    import yaml
    with open(DECL) as f:
        return yaml.safe_load(f)["distros"]


def find_packages():
    """Every find_package() in CMakeLists.txt, WITH whether it is REQUIRED.

    ⚑ THE KEYWORD IS READ, AND THE FIRST VERSION OF THIS FUNCTION DID NOT READ IT.
    It returned names only. On 2026-09-07, while the port to jazzy/kilted was in
    progress, `find_package(nav2_ros_common REQUIRED)` became
    `find_package(nav2_ros_common QUIET)` -- and a name-only probe would have gone
    on reporting `nav2_ros_common MISSING` on jazzy, and gone on calling the floor
    intact, for as long as anyone cared to look. That is the mean-direction failure
    this whole file was written to catch, rebuilt inside the instrument meant to
    catch it. A dependency CMake does not require cannot hold a floor up.

    The list is read from the file and never written here: a dependency list in
    this script would be a second copy of a fact that can drift.
    """
    txt = open(os.path.join(ROOT, "CMakeLists.txt")).read()
    seen, out = set(), []
    for m in re.finditer(r"^\s*find_package\(\s*([A-Za-z0-9_]+)([^)]*)\)", txt, re.M):
        name, rest = m.group(1), m.group(2)
        if name in seen:
            continue
        seen.add(name)
        out.append((name, "REQUIRED" in rest))
    return out


def tree_identity():
    """What tree was this measured on — stated, never assumed.

    ⚑ THIS EXISTS BECAUSE A MEASUREMENT WAS REPORTED THAT COULD NOT BE ATTRIBUTED.
    On 2026-09-07 this seat ran the floor probe on jazzy, got a clean pass, and
    reported it -- on a working tree carrying seventeen uncommitted files of
    somebody else's in-progress port, including the very `package.xml` line the
    probe was measuring. The number was true of a tree that exists on no branch
    and in no clone but one. A green with no tree identity is not a result; it is
    an anecdote, and the reader cannot tell which.
    """
    env = os.environ.get("FLOOR_TREE_ID")
    if env:
        return env
    try:
        sha = subprocess.run(["git", "-C", ROOT, "rev-parse", "HEAD"],
                             capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", ROOT, "status", "--porcelain"],
                               capture_output=True, text=True, check=True).stdout
    except Exception:
        return ("UNKNOWN — no git and no FLOOR_TREE_ID. This measurement cannot be "
                "attributed to a tree and must not be quoted as one.")
    lines = [l for l in dirty.split("\n") if l.strip()]
    if not lines:
        return "%s (clean)" % sha[:7]
    import hashlib
    d = hashlib.sha256(dirty.encode()).hexdigest()[:12]
    return ("%s + %d uncommitted paths [%s] — NOT a published tree"
            % (sha[:7], len(lines), d))


def probe_dependencies():
    """Which find_package() calls can this substrate satisfy, one at a time?"""
    results = []
    with tempfile.TemporaryDirectory() as tmp:
        for pkg, required in find_packages():
            src = os.path.join(tmp, pkg)
            os.makedirs(os.path.join(src, "b"), exist_ok=True)
            with open(os.path.join(src, "CMakeLists.txt"), "w") as f:
                f.write("cmake_minimum_required(VERSION 3.10)\nproject(probe)\n"
                        "find_package(%s REQUIRED)\n" % pkg)
            rc = subprocess.run(["cmake", "-S", src, "-B", os.path.join(src, "b")],
                                capture_output=True, text=True).returncode
            results.append((pkg, required, rc == 0))
    return results


def configure_real():
    """Configure the ACTUAL package. This is the claim; the probe above is the diagnosis.

    ⚑ THE PER-DEPENDENCY PROBE IS NOT THE FLOOR. It names which door is locked,
    which is what a porter needs; but a package can be missing an optional
    dependency and build perfectly well. What "does it build here" MEANS is what
    CMake says about CMakeLists.txt, so that is what is asked.
    """
    with tempfile.TemporaryDirectory() as b:
        r = subprocess.run(["cmake", "-S", ROOT, "-B", b], capture_output=True, text=True)
        return r.returncode == 0, (r.stdout + r.stderr)


def cmd_probe(distro):
    decl = {d["name"]: d for d in declaration()}
    if distro not in decl:
        print("FLOOR: '%s' is not declared in %s. Add it there; do not special-case it "
              "here." % (distro, os.path.relpath(DECL, ROOT)), file=sys.stderr)
        return 1
    d = decl[distro]
    supported = d["status"] == "supported"
    expect_missing = set(d.get("expect_missing") or [])

    print("floor probe — %s (nav2 %s, Ubuntu %s), declared %s"
          % (distro, d["nav2"], d["ubuntu"], d["status"]))
    print("tree: %s" % tree_identity())

    deps = probe_dependencies()
    print("  find_package(), as CMakeLists.txt declares them:")
    for pkg, required, ok in deps:
        print("    %-22s %-9s %s" % (pkg, "REQUIRED" if required else "optional",
                                     "ok" if ok else "MISSING"))
    required_missing = {p for p, req, ok in deps if req and not ok}
    optional_missing = sorted(p for p, req, ok in deps if not req and not ok)

    configured, clog = configure_real()
    print("  configure (cmake on the real CMakeLists.txt): %s"
          % ("SUCCEEDED" if configured else "FAILED"))
    if not configured:
        for line in clog.split("\n"):
            if "CMake Error" in line or "Could not find a package" in line:
                print("    | %s" % line.strip())

    print("  declared missing : %s" % (sorted(expect_missing) or "(none)"))
    print("  REQUIRED missing : %s" % (sorted(required_missing) or "(none)"))
    if optional_missing:
        print("  optional missing : %s  (does not hold a floor up)" % optional_missing)

    problems = []
    if supported and not configured:
        problems.append("%s is declared SUPPORTED and the package does not configure "
                        "on it." % distro)
    if not supported and configured:
        problems.append(
            "⚑ %s is declared UNSUPPORTED and the package CONFIGURES on it.\n"
            "   README.md tells a %s developer this package has nothing for her, and\n"
            "   that is no longer true. She is being turned away for nothing, and no\n"
            "   other check in this repository would ever have said so.\n"
            "   Set `status: supported` in %s, clear `expect_missing`, and re-render\n"
            "   the README. Do not soften the declaration to keep this light green."
            % (distro, distro, os.path.relpath(DECL, ROOT)))
    if required_missing != expect_missing:
        unexpected = sorted(required_missing - expect_missing)
        resolved = sorted(expect_missing - required_missing)
        if unexpected:
            problems.append(
                "this substrate is missing a REQUIRED dependency nobody declared: %s.\n"
                "   Either CMakeLists.txt gained something this distribution cannot\n"
                "   satisfy, or the feed moved. Both need a human." % ", ".join(unexpected))
        if resolved:
            problems.append(
                "the declaration still calls these missing on %s, and they are not: %s.\n"
                "   Most often a find_package() stopped being REQUIRED, or a\n"
                "   compatibility path landed. The declaration is stale either way."
                % (distro, ", ".join(resolved)))

    if not problems:
        print("FLOOR HOLDS: the package %s configure here, exactly as declared."
              % ("does" if configured else "does not"))
        return 0
    print("", file=sys.stderr)
    print("FLOOR BROKEN.", file=sys.stderr)
    for p in problems:
        print(" - %s" % p, file=sys.stderr)
    return 1


def render():
    rows = ["| ROS 2 | Ubuntu | nav2 | builds today |",
            "|---|---|---|---|"]
    for d in declaration():
        ok = d["status"] == "supported"
        note = "**yes**" if ok else "no — missing `%s`" % "`, `".join(d.get("expect_missing") or ["?"])
        rows.append("| `%s` | %s | `%s` | %s |" % (d["name"], d["ubuntu"], d["nav2"], note))
    return "\n".join(
        [BEGIN,
         "",
         "<!-- Generated. Every row is proved on every run by the `distro floor` CI job,",
         "     in both directions: a `no` that starts building reddens the job too. -->",
         ""] + rows + [
         "",
         "Measured against the live package feed, not inferred from headers.",
         "",
         END])


def cmd_render(write):
    block = render()
    txt = open(RDM).read()
    if BEGIN in txt and END in txt:
        pre = txt[:txt.index(BEGIN)]
        post = txt[txt.index(END) + len(END):]
        new = pre + block + post
    else:
        print("README.md carries no generated block. Insert these markers where the "
              "table belongs:\n  %s\n  %s" % (BEGIN, END), file=sys.stderr)
        print(block)
        return 2
    if not write:
        if new == txt:
            print("README floor table matches %s." % os.path.relpath(DECL, ROOT))
            return 0
        print("README floor table has DRIFTED from %s." % os.path.relpath(DECL, ROOT),
              file=sys.stderr)
        print("Run:  python3 scripts/distro_floor.py --render --write", file=sys.stderr)
        import difflib
        for line in difflib.unified_diff(txt.split("\n"), new.split("\n"),
                                         "README.md (in tree)", "README.md (generated)",
                                         lineterm="", n=1):
            print(line, file=sys.stderr)
        return 1
    open(RDM, "w").write(new)
    print("README floor table written from %s." % os.path.relpath(DECL, ROOT))
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--matrix", action="store_true")
    ap.add_argument("--probe", metavar="DISTRO")
    ap.add_argument("--render", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--write", action="store_true")
    a = ap.parse_args()
    if a.matrix:
        print(json.dumps({"include": declaration()}))
        return 0
    if a.probe:
        return cmd_probe(a.probe)
    if a.check:
        return cmd_render(write=False)
    if a.render:
        return cmd_render(write=a.write)
    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
