#!/usr/bin/env bash
# ════════════════════════════════════════════════════════════════════════════════
# hubot LAUNCH GATE — the package's own bring-up, run as a stranger would run it.
#
# WHY THIS FILE IS IN THE REPOSITORY.
# A costmap filter is five things; hubot ships one and tells the edge developer to
# build the other four. This gate is what proves the other four work when SHE runs
# them rather than when we do. It was born as an FBR clean-room harness that lived
# outside this package — which meant the only complete gate for hubot could not be
# run by anyone who cloned hubot. A gate a contributor cannot run is a gate she is
# asked to take on trust, and taking our verification on trust is the thing she
# opened the pull request to avoid.
#
# ⚑ WHAT A GREEN RUN LICENSES, STATED BEFORE THE RUN SO IT CANNOT BE INFLATED AFTER.
#   ONE substrate: Ubuntu 26.04 amd64, ROS lyrical, rmw_fastrtps_cpp, nav2 1.5.x from
#   packages.ros.org. hubot's own launch file brings a costmap filter up and the four
#   cases report differently in the way the CHANGELOG says they do.
#   It does NOT license "an integrator can install it."
#   THE ROBOT IS PLACED, NOT DRIVEN: robot_x/robot_y feed a static_transform_publisher.
#   Nothing moves. plugins: ["static_layer"] only — no obstacle layer, no sensors, no
#   simulator. This is BRING-UP evidence, not robot evidence.
#
# ⚑ HONEST DELTA FROM THE FBR CLEAN ROOM, NAMED RATHER THAN GLOSSED.
#   FBR's harness proves the repository is ABSENT FROM THE FILESYSTEM during the four
#   launches (docker commit does not capture bind mounts, so /ws/src/hubot is empty).
#   This gate cannot claim that: in CI the checkout necessarily remains on disk. What
#   this gate proves instead is the weaker, true thing — the launch file, parameters and
#   maps that the runs resolve are the INSTALLED ones under the install prefix, asserted
#   by step 2, not the checkout's. "The repository is absent" and "the install space is
#   what was read" are different claims and only the second is made here.
#
# usage:  test/launch_gate.sh <install-prefix> [--selftest]
#   --selftest  assert a DELIBERATELY WRONG expectation on one case and require this
#               gate to report DID NOT REPRODUCE. A gate that has never failed has
#               measured nothing.
#
# exit 0 reproduced · 2 did-not-reproduce · 3 environment-incomplete
# ════════════════════════════════════════════════════════════════════════════════
set -u
PREFIX="${1:?usage: launch_gate.sh <install-prefix> [--selftest]}"
SELFTEST=0; [ "${2:-}" = "--selftest" ] && SELFTEST=1
LOGS="${LAUNCH_GATE_LOGS:-/tmp/hubot_launch_gate}"; mkdir -p "$LOGS"
R="$LOGS/RESULT.txt"; : > "$R"
say(){ echo "$*" | tee -a "$R"; }
FAILED=0

[ -f "$PREFIX/setup.bash" ] || { say "ENVIRONMENT-INCOMPLETE: no setup.bash under $PREFIX"; exit 3; }
# ⚑ `set -u` OFF ACROSS THE SOURCE, ON PURPOSE. colcon's own setup.bash dereferences
# COLCON_TRACE unguarded; under `set -u` sourcing it aborts this gate with
# "COLCON_TRACE: unbound variable" — a RED naming a variable the contributor never
# wrote, about a file she does not own. A gate must not fail for a reason its reader
# cannot act on. Found by running this gate, 2026-09-07, not by reading it.
set +u
# shellcheck disable=SC1090,SC1091
source "$PREFIX/setup.bash"
set -u
command -v ros2 >/dev/null || { say "ENVIRONMENT-INCOMPLETE: ros2 CLI absent"; exit 3; }
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

say "== 1. what this gate is about to measure =="
say "install prefix : $PREFIX"
say "nav2_costmap_2d: $(dpkg -s ros-lyrical-nav2-costmap-2d 2>/dev/null | awk -F': ' '/^Version/{print $2}')"
say "rmw            : ${RMW_IMPLEMENTATION:-<default>}"

say "== 2. the runs resolve the INSTALLED launch file, not the checkout =="
SHARE="$(ros2 pkg prefix hubot 2>/dev/null)/share/hubot" || true
LAUNCHFILE="$SHARE/launch/zone_filter_demo_launch.py"
say "resolved share dir: $SHARE"
if [ ! -f "$LAUNCHFILE" ]; then
  say "ENVIRONMENT-INCOMPLETE: installed launch file absent at $LAUNCHFILE"; exit 3
fi
case "$LAUNCHFILE" in
  "$PREFIX"/*) say "installed launch file: $LAUNCHFILE  (under the install prefix — OK)";;
  *) say "⚑ the resolved launch file is NOT under $PREFIX -- $LAUNCHFILE"; FAILED=1;;
esac
say "params installed: $(ls "$SHARE/params" 2>/dev/null | tr '\n' ' ')"
say "maps installed  : $(ls "$SHARE/maps" 2>/dev/null | tr '\n' ' ')"

# ⚑ WHY EACH CASE IS ISOLATED, AND HOW THIS WAS LEARNED.
# FBR's original harness ran every case in its OWN CONTAINER. This gate runs them in
# one process, so that isolation had to be rebuilt — and the first version did not
# rebuild it. Measured 2026-09-07: run sequentially with only `pkill -f "ros2 launch"`
# between them, the `outside_zone` case reported arrivals=2. Run ALONE in a fresh
# container the same case reports arrivals=1. `ros2 launch` spawns children that
# killing the parent does not reap, and a survivor from the previous case contaminates
# the next one. Container-per-case was load-bearing, not incidental.
#
# Three defences, because one was not enough:
#   1. a distinct ROS_DOMAIN_ID per case — a survivor cannot even see the next case
#   2. setsid + process-group kill + wait — the children are actually reaped
#   3. ⚑ A CONTAMINATION ASSERTION BEFORE EVERY CASE. If anything survived, this gate
#      says so and exits 3 (environment-incomplete) rather than reporting a number it
#      cannot trust. A harness that silently reports a contaminated count is worse than
#      one that refuses: the contaminated count looks exactly like a finding.
# ⚑ THE GUARD REFUSES TO RUN WITHOUT ITS INSTRUMENT, RATHER THAN PASSING WITHOUT IT.
# assert_clean below is built on pgrep, which is in `procps` and is NOT present in a
# bare ubuntu:26.04. With pgrep absent, `pgrep ... | wc -l` returns 0 and the guard
# reports CLEAN on every call — a contamination check that has quietly become a no-op
# while still printing a reassuring line. Found 2026-09-07 while wiring this into CI,
# by installing the substrate from scratch rather than reusing one that happened to
# have procps. An absent verdict must never read as a pass.
command -v pgrep >/dev/null || {
  say "ENVIRONMENT-INCOMPLETE: pgrep absent (package: procps)."
  say "  This gate's contamination check is built on pgrep. Without it the check would"
  say "  report CLEAN unconditionally, so this gate refuses rather than measuring blind."
  exit 3; }

DOMAIN=41
assert_clean(){
  local lbl="$1" leftover
  leftover=$(pgrep -f "zone_target_demo_node|nav2_costmap_2d|map_server|lifecycle_manager|ros2 launch" 2>/dev/null | wc -l)
  if [ "$leftover" -ne 0 ]; then
    say "⚑ CONTAMINATED before case '$lbl': $leftover process(es) survived the previous case."
    say "   This gate will not report a number it cannot trust. Survivors:"
    pgrep -af "zone_target_demo_node|nav2_costmap_2d|map_server|lifecycle_manager|ros2 launch" 2>/dev/null | sed 's/^/     /' | tee -a "$R"
    exit 3
  fi
}

teardown(){
  local pgid="$1"
  kill -TERM "-$pgid" 2>/dev/null
  local i=0
  while [ $i -lt 15 ] && pgrep -f "zone_target_demo_node|nav2_costmap_2d|ros2 launch" >/dev/null 2>&1; do
    sleep 1; i=$((i+1))
  done
  kill -KILL "-$pgid" 2>/dev/null
  sleep 2
}

# case: label | launch args | expected enforced | expected arrivals
run_case(){
  local lbl="$1" args="$2" exp_enf="$3" exp_arr="$4"
  assert_clean "$lbl"
  DOMAIN=$((DOMAIN+1)); export ROS_DOMAIN_ID=$DOMAIN
  local d="$LOGS/$lbl"; mkdir -p "$d"; rm -f "$d"/*.log
  setsid bash -c "ros2 launch hubot zone_filter_demo_launch.py $args > '$d/launch.log' 2>&1" &
  local pgid=$!
  sleep "${LAUNCH_GATE_SETTLE:-28}"
  # ⚑ --no-daemon IS LOAD-BEARING, and this is not style. With the ros2 CLI daemon,
  # `topic list` on an isolated network returned a STALE graph missing /map,
  # /zone_filter_mask AND /zone_decision while all three were being published — a
  # healthy stack reading as a dead one. The instrument is part of the measurement.
  timeout 20 ros2 topic echo --no-daemon /zone_decision diagnostic_msgs/msg/DiagnosticArray --once > "$d/decision.log" 2>&1
  local echo_exit=$?
  local enf arr rr msg
  enf=$(grep -A1 "key: enforced\$" "$d/decision.log" | grep -m1 "value:" | sed "s/.*value: //" | tr -d "'")
  arr=$(grep -c "ARRIVED HERE" "$d/launch.log")
  rr=$(timeout 15 ros2 param get --no-daemon /local_costmap/costmap robot_radius 2>&1 | tail -1)
  msg=$(grep -m1 "  message:" "$d/decision.log" | sed "s/^ *message: //")
  teardown "$pgid"
  say "-- $lbl -- args='${args:-<none>}' (ROS_DOMAIN_ID=$DOMAIN)"
  say "     echo_exit=$echo_exit"
  say "     message=$msg"
  say "     enforced=$enf  arrivals=$arr  costmap_robot_radius=$rr"
  if [ "$enf" = "$exp_enf" ] && [ "$arr" = "$exp_arr" ]; then
    say "     VERDICT: REPRODUCED (expected enforced=$exp_enf arrivals=$exp_arr)"
  else
    # ⚑ THE FAILURE SENTENCE NAMES WHAT WAS EXPECTED, WHAT ARRIVED, AND WHERE TO LOOK.
    # A contributor who cannot tell from a RED what she broke has been handed our
    # verification, not relieved of it.
    say "     ⚑ VERDICT: DID NOT REPRODUCE"
    say "        case          : $lbl  (launch args: '${args:-<none>}')"
    say "        expected      : enforced=$exp_enf  arrivals=$exp_arr"
    say "        got           : enforced=$enf  arrivals=$arr"
    say "        what this means: 'enforced' is the filter's own report on /zone_decision."
    say "                         'arrivals' counts 'ARRIVED HERE' in the demo target node —"
    say "                         i.e. whether the parameter set actually landed on the target."
    say "        full launch log: $d/launch.log"
    say "        decision dump  : $d/decision.log"
    FAILED=1
  fi
}

say "== 3. the four cases =="
if [ $SELFTEST -eq 1 ]; then
  say "⚑ SELFTEST: asserting a KNOWINGLY WRONG expectation on ctl_readonly."
  say "⚑ This gate must report DID NOT REPRODUCE and exit 2. If it exits 0, the gate is blind."
  run_case ctl_readonly "overlay:=overlay_target_readonly.yaml" "yes" 1
  if [ $FAILED -eq 1 ]; then
    say "SELFTEST PASSED: the gate reported the mismatch it was given."; exit 0
  else
    say "⚑ SELFTEST FAILED: the gate accepted a wrong expectation. It is measuring nothing."; exit 2
  fi
fi

run_case base          ""                                            "yes" 1
run_case outside_zone  "robot_x:=2.0 robot_y:=2.0"                    "yes" 1
run_case ctl_readonly  "overlay:=overlay_target_readonly.yaml"        "NO"  0
run_case ctl_inside_ns "overlay:=overlay_target_inside_namespace.yaml" "NO"  0

say "== DONE $(date -u +%FT%TZ) =="
if [ $FAILED -eq 0 ]; then
  say "JUDGMENT: REPRODUCED on this ONE substrate. Not 'an integrator can install it'."
  say "          The robot was placed, not driven."
  exit 0
fi
say "JUDGMENT: DID NOT REPRODUCE — see the ⚑ lines above."
exit 2
