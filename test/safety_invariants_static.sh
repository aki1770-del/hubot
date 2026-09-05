#!/usr/bin/env bash
# hubot — SOTIF static safety invariants.  Author: FSE, 2026-09-05.
#
# ⚑ WHY THIS EXISTS, WRITTEN BEFORE THE ACT (OPS-070(B)).
#
# hubot's central design decision is that it DEGRADES where upstream ABORTS.
# Every existing test asks "can we prove it degrades?".  None asks whether the
# thing it degrades INTO is a state a person can act on, or whether the abort
# path is actually gone.  Both questions are answerable WITHOUT a ROS
# toolchain, by reading the committed source, and both are currently answered
# NO.
#
# It is deliberately toolchain-free: bash + grep + awk.  The behavioural suite
# needs colcon, nav2, rclcpp, a live executor and a second node, so on any host
# without ROS it cannot run at all -- and "the suite did not run" reads exactly
# like "the suite passed" (MEMORY 2026-08-20: an absent verdict reads as a
# pass).  This gate always runs, everywhere, in under a second.
#
# Sakichi Vision 14, resolved this turn:
#   "Silent failure is the anti-Jidoka -- a function that returns a
#    success-shaped value while the operation failed is a loom weaving through
#    a broken warp."
# Sakichi Vision 20, resolved this turn:
#   "Stopping must be cheap, or operators will hesitate; design the halt to
#    cost less than the defect."
#
# HONEST BOUND, stated here and not in a commit message: this is a STRUCTURAL
# gate.  It proves the shape of the source, never the behaviour of the robot.
# INV-A does not prove no exception escapes process(); it proves no `throw`
# token is written in the closure of functions process() calls in THIS
# translation unit.  A callee in nav2 or rclcpp that throws is invisible to it.
# It is a sound partial and is not a substitute for the gtest suite.
#
# Usage:  bash hubot/test/safety_invariants_static.sh [<hubot-dir>]
# Exit:   0 all invariants hold  ·  1 any invariant violated  ·  2 substrate error

set -uo pipefail

HUBOT_DIR="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
SRC="$HUBOT_DIR/src/zone_parameter_filter.cpp"
HDR="$HUBOT_DIR/include/hubot/zone_parameter_filter.hpp"

for f in "$SRC" "$HDR"; do
  [[ -f "$f" ]] || { printf 'SUBSTRATE ERROR: missing %s\n' "$f"; exit 2; }
done

fail=0
pass_one() { printf 'PASS  %-8s %s\n' "$1" "$2"; }
fail_one() { printf 'FAIL  %-8s %s\n' "$1" "$2"; fail=1; }

# Extract one function body: from the line whose text matches the signature,
# to the next `}` in column 0.  Relies on the file's uncrustify-clean layout.
body_of() {
  awk -v sig="$1" '
    index($0, sig) && !started { started = 1 }
    started { print NR ":" $0 }
    started && /^}/ { exit }
  ' "$SRC"
}

# The transitive closure of what process() calls inside this translation unit.
# Enumerated by reading process() and each callee, not guessed.
PROCESS_CLOSURE=(
  "void ZoneParameterFilter::process("
  "void ZoneParameterFilter::applyState("
  "void ZoneParameterFilter::resetToNominal("
  "void ZoneParameterFilter::issueAsyncSetParameters("
  "void ZoneParameterFilter::checkPendingParameterUpdates("
  "void ZoneParameterFilter::publishDecision("
)

# ---------------------------------------------------------------------------
# INV-A  NO ABORT PATH SURVIVES ON THE COSTMAP THREAD.
#
# CostmapFilter::updateCosts() is bare and LayeredCostmap has no handler
# anywhere -- the source states this itself at zone_parameter_filter.cpp:513.
# Anything that throws from the closure of process() is std::terminate, i.e.
# the abort this package exists to replace.  A degrade policy with one
# surviving throw has not replaced the abort; it has narrowed it, and narrowed
# it to a path reached by RUNTIME MASK DATA rather than by config -- which is
# strictly harder to catch before the robot is in the field.
# ---------------------------------------------------------------------------
inv_a() {
  local hits="" fn
  for fn in "${PROCESS_CLOSURE[@]}"; do
    while IFS= read -r line; do
      # Skip comment lines: a `throw` discussed in prose is not a throw.
      case "${line#*:}" in
        *"//"*"throw"*) continue ;;
      esac
      case "$line" in
        *"throw "*|*"throw("*) hits+="${line%%:*} " ;;
      esac
    done < <(body_of "$fn")
  done
  if [[ -z "$hits" ]]; then
    pass_one "INV-A" "no throw in the process() closure -- the abort path is gone"
  else
    fail_one "INV-A" "throw reachable from process() at src line(s): ${hits% } -- reaches CostmapFilter::updateCosts(), which is bare. This IS the upstream abort, still present."
  fi
}

# ---------------------------------------------------------------------------
# INV-B  THE HUMAN SURFACE MUST NOT PUBLISH A ZONE IT HAS NOT ENTERED.
#
# publishDecision() renders `current_state_`.  If process() publishes before
# assigning it, the DiagnosticArray names the PREVIOUS zone at the exact moment
# the source's own comment says "a transition is exactly when a human decision
# is due".  Entering a danger zone would print "Outside any zone; nominal
# defaults are in force."  That is not a missing warning -- it is the opposite
# of the truth, delivered to the person deciding whether to trust the robot.
# ---------------------------------------------------------------------------
inv_b() {
  local body assign_ln pub_ln
  body="$(body_of "void ZoneParameterFilter::process(")"
  assign_ln="$(grep -m1 'current_state_ = new_state;' <<<"$body" | cut -d: -f1)"
  pub_ln="$(grep -m1 'publishDecision("zone transition")' <<<"$body" | cut -d: -f1)"
  if [[ -z "$assign_ln" || -z "$pub_ln" ]]; then
    fail_one "INV-B" "could not locate the transition assignment and/or publish in process() -- the invariant cannot be evaluated, which is not a pass"
    return
  fi
  if (( assign_ln < pub_ln )); then
    pass_one "INV-B" "current_state_ assigned at :$assign_ln before publishDecision at :$pub_ln"
  else
    fail_one "INV-B" "publishDecision at :$pub_ln runs BEFORE current_state_ is assigned at :$assign_ln -- every transition diagnostic names the zone the robot just LEFT"
  fi
}

# ---------------------------------------------------------------------------
# INV-C  A DECLARED BOUND MUST ACTUALLY BOUND.
#
# The header declares kMaxPendingSets = 64 with the comment "Bounds in-flight
# sets against a target that never answers."  If nothing references it, the
# most likely field failure -- a target node that is dead, unstarted, or
# misnamed -- produces a future that never becomes ready, no failure, no
# latch, and `enforced: yes` forever.  Vision 14, in the one feature built to
# abolish Vision 14.
# ---------------------------------------------------------------------------
#
# ⚑ CPP 2026-09-05 (authored by CPP on FSE's gate, reported to FSE + CT, not
# silently changed): the first cut was `grep -q 'kMaxPendingSets' "$SRC"`, which
# passes on a MENTION -- including a comment saying the bound is not enforced.
# A guard that a comment can satisfy is not a guard. Comment lines are stripped
# before the check now.
inv_c() {
  if grep -v '^[[:space:]]*//' "$SRC" | grep -q 'kMaxPendingSets'; then
    pass_one "INV-C" "kMaxPendingSets is referenced by CODE (not merely a comment) in the source"
  else
    fail_one "INV-C" "kMaxPendingSets is declared at $(grep -n 'kMaxPendingSets' "$HDR" | cut -d: -f1) and referenced 0 times in the source -- in-flight sets are unbounded and a never-answering target is never detected"
  fi
}

# ---------------------------------------------------------------------------
# INV-D  A RELOAD MUST NOT CARRY THE PREVIOUS CONFIGURATION'S RESULTS.
#
# resetFilter() clears enforcement_degraded_ because "the configuration it
# referred to is gone".  If pending_futures_ is not also dropped, results
# issued against the OLD configuration land after the reload and re-latch the
# flag against a configuration they never touched -- or, worse, are counted as
# confirmation of it.  checkPendingParameterUpdates() runs at the TOP of
# process(), before the mask check, so it drains them on the very next cycle.
# ---------------------------------------------------------------------------
#
# ⚑ CPP 2026-09-05 (authored by CPP on FSE's gate; reported to FSE + CT rather
# than changed quietly). THIS CHECK WAS A LITERAL GREP FOR `pending_futures_`
# INSIDE resetFilter(). That names an IMPLEMENTATION VARIABLE, not the property,
# so it went wrong in both directions at once: it reported a FAILURE against a
# resetFilter() that does clear its in-flight sets under a different member
# name, and it would have reported a PASS for any rename that kept the bug.
# The container is now derived from the header, so the check follows the code.
# (The old failure text also asserted "resetFilter() clears
# enforcement_degraded_", which stopped being true on the same day -- clearing
# it there was AAA's N-1 finding and the clear was removed.)
inv_d() {
  local body members m missing="" found=""
  body="$(body_of "void ZoneParameterFilter::resetFilter(")"
  # Every in-flight-set container the HEADER declares, whatever it is called.
  members="$(grep -oE '\bpending[A-Za-z0-9_]*_\b' "$HDR" | sort -u)"
  if [[ -z "$members" ]]; then
    fail_one "INV-D" "no in-flight-set container found in the header -- the invariant cannot be evaluated, which is not a pass"
    return
  fi
  for m in $members; do
    if grep -v '^[[:space:]]*[0-9]*:[[:space:]]*//' <<<"$body" | grep -q "${m}\.clear()"; then
      found+="$m "
    else
      missing+="$m "
    fi
  done
  if [[ -z "$missing" ]]; then
    pass_one "INV-D" "resetFilter() drops every in-flight-set container the header declares: ${found% }"
  else
    fail_one "INV-D" "resetFilter() does not clear: ${missing% } -- results issued against the DISCARDED configuration land after the reload and are attributed to the new one"
  fi
}

# ---------------------------------------------------------------------------
# INV-E  NO DECLARED-BUT-ABSENT SAFETY SURFACE.
#
# A member or method declared in the public header and defined nowhere is a
# promise to an integrator that the binary cannot keep.  reapplyAfterDrainIfDue()
# even carries a written RACE-FREEDOM ARGUMENT in its doc comment -- a safety
# argument for a function that does not exist.  An integrator reading the
# header to decide whether to trust the filter reads that argument as fact.
# ---------------------------------------------------------------------------
inv_e() {
  local sym missing=""
  for sym in enterState reapplyAfterDrainIfDue reapply_after_drain_ pending_sets_ PendingSet; do
    grep -q "$sym" "$HDR" || continue
    grep -q "$sym" "$SRC" || missing+="$sym "
  done
  if [[ -z "$missing" ]]; then
    pass_one "INV-E" "every declared symbol checked is referenced in the source"
  else
    fail_one "INV-E" "declared in the header, absent from the source: ${missing% } -- the class does not hold together and the header over-promises"
  fi
}

# ---------------------------------------------------------------------------
# INV-F  THE HEADER MUST DOCUMENT THE POLICY THE CODE IMPLEMENTS.
#
# The header's doc comment for checkPendingParameterUpdates() still states the
# UPSTREAM policy: "a failed set on any target throws (a failed set on a safety
# parameter is a stop condition)."  The code latches and continues.  The
# integrator's primary interface document therefore describes the exact
# behaviour this package was built to reverse -- and it is the one sentence an
# integrator would use to decide they need no fallback of their own.
# ---------------------------------------------------------------------------
#
# ⚑ FSE 2026-09-05: the first cut of this check grepped the WHOLE header for
# "throws" and reported header:110 -- which documents applyState() throwing on
# an unknown state, and is ACCURATE.  The message it printed ("the code latches
# and continues") was false about that line.  A gate that fires on the wrong
# evidence teaches the next reader to distrust it, so it is scoped to the doc
# block that actually precedes checkPendingParameterUpdates().  Recorded here
# rather than silently corrected.  Note what the mis-fire incidentally proved:
# the header states applyState() throws, so INV-A's finding is DOCUMENTED
# behaviour, not an oversight.
inv_f() {
  local ln
  ln="$(awk '
    /^[[:space:]]*\/\*\*/ { blk_start = NR; delete blk; n = 0 }
    { blk[n++] = NR ":" $0 }
    /void checkPendingParameterUpdates\(/ {
      for (i = 0; i < n; i++) {
        split(blk[i], p, ":")
        if (p[1] >= blk_start && blk[i] ~ /throws/) { print p[1]; exit }
      }
      exit
    }
  ' "$HDR")"
  if [[ -z "$ln" ]]; then
    pass_one "INV-F" "checkPendingParameterUpdates()'s doc block does not claim a throw policy the code no longer implements"
  else
    fail_one "INV-F" "header:$ln documents checkPendingParameterUpdates() as 'throws' -- the code latches and continues. The integrator's primary interface document states the policy this package exists to reverse, and it is the sentence they would use to decide they need no fallback of their own."
  fi
}

printf '\nhubot SOTIF static safety invariants — FSE — %s\n' "$(date +%Y-%m-%d)"
printf 'src: %s\nhdr: %s\n\n' "$SRC" "$HDR"

inv_a; inv_b; inv_c; inv_d; inv_e; inv_f

printf '\n'
if (( fail )); then
  printf '⚑ VIOLATED. These are structural facts of the committed source, not opinions.\n'
  printf '  Each line above names the file position that refutes the invariant.\n'
  exit 1
fi
printf 'All static safety invariants hold.\n'
exit 0
