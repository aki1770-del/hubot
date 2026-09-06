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

# ⚑ THE PREDICATE AND ITS CONTROLS SIT ABOVE THE ROS CHECKS ON PURPOSE: they test
# process bookkeeping, not a built workspace, so a contributor can run
#   test/launch_gate.sh . --selftest-contamination
# on a bare machine with nothing installed and see the guard prove itself.

# ⚑ THE GUARD REFUSES TO RUN WITHOUT ITS INSTRUMENT, RATHER THAN PASSING WITHOUT IT.
# The contamination check below is built on ps/pgrep, in `procps`, which is NOT present
# in a bare ubuntu:26.04. With them absent the check would report CLEAN on every call --
# a contamination check quietly turned into a no-op while still printing a reassuring
# line. An absent verdict must never read as a pass.
for _t in ps pgrep; do
  command -v "$_t" >/dev/null || {
    say "ENVIRONMENT-INCOMPLETE: $_t absent (package: procps)."
    say "  This gate's contamination check is built on it. Without it the check would"
    say "  report CLEAN unconditionally, so this gate refuses rather than measuring blind."
    exit 3; }
done

# Processes that would genuinely contaminate the next case if they were still running.
# ⚑ EVERY ALTERNATIVE'S FIRST CHARACTER IS BRACKETED, AND THAT IS NOT DECORATION.
# `ps ... | grep -E "$PROCPAT"` puts the pattern into the grep's OWN command line, which
# `ps` then lists, which the grep then matches -- the instrument counting itself. Measured
# 2026-09-07: the predicate reported 2 survivors on an idle host with nothing spawned, and
# the two were its own grep. A guard with a permanent non-zero floor refuses forever.
# `[n]av2_costmap_2d` matches the string "nav2_costmap_2d" and does NOT match the literal
# "[n]av2_costmap_2d" sitting in the grep's argv, so the instrument drops out of its own
# measurement. The bare form is kept below for the human reader.
#   readable: zone_target_demo_node|nav2_costmap_2d|map_server|lifecycle_manager|ros2 launch
PROCPAT='[z]one_target_demo_node|[n]av2_costmap_2d|[m]ap_server|[l]ifecycle_manager|ros2 [l]aunch'

# ⚑ THE PREDICATE IS *LIVE*, NOT *PRESENT*, AND THIS COST A RED CI RUN TO LEARN.
# The first version of this guard counted anything pgrep matched. On GitHub's runner the
# first run reported three survivors before the very first case -- 8175 [map_server]
# <defunct>, 8176 [map_server] <defunct>, 8178 [nav2_costmap_2d] <defunct>. All three
# were ZOMBIES: already dead, merely un-reaped, because a container whose PID 1 is a
# shell reaps no orphans. A defunct process holds no sockets, joins no DDS graph and
# cannot influence any count -- so the guard refused over a condition that could not
# contaminate anything, and a gate that refuses on a healthy tree measures as little as
# one that passes on a broken one.
# The state column is the discriminator: Linux marks a zombie Z, and `ps` reports it.
# ⚑ NOT FIXED WITH `--init`, AND NOT WITH A wait() IN TEARDOWN. A reaping init does remove
# the zombies, but it is a property of HOW THE CONTAINER WAS STARTED, and this script has to
# be right when a contributor runs it on her own machine with no container at all: a guard
# whose correctness depends on the caller's runtime flags is a guard that lies on her laptop.
# A wait() in teardown cannot work at all -- a process may only wait() for its own children,
# and these are the launch file's children, reparented to PID 1 when their parent died. They
# were never ours to reap. The predicate is the only fix that is correct everywhere.
live_survivors(){
  ps -eo pid=,stat=,args= 2>/dev/null \
    | awk '$2 !~ /Z/' \
    | grep -vw "defunct" \
    | grep -E "$PROCPAT" \
    | grep -v "launch_gate.sh"
}

# ⚑ THE PREDICATE CARRIES ITS OWN PROOF, IN BOTH DIRECTIONS.
# Narrowing a guard is how a guard becomes a no-op, so narrowing it without a control
# that still fires would be trading one blind gate for another. These two run the real
# predicate against the two states it must tell apart:
#   A  a genuinely LIVE process matching the pattern  -> must still refuse
#   B  a genuine ZOMBIE matching the pattern          -> must NOT refuse
# Control B creates a real zombie the way the runner did: a child that exits under a
# parent which never waits.
selftest_contamination(){
  local base rc_live rc_zomb zcount out
  set +m                      # no "Killed" job-control chatter in the transcript
  say "== contamination predicate, both directions =="
  base=$(live_survivors | grep -c . || true)
  say "     host baseline (0 once the predicate stops matching its own grep): $base"

  # ---- A: a genuinely LIVE matching process MUST be refused ----
  setsid bash -c 'exec -a nav2_costmap_2d_livectl sleep 25' >/dev/null 2>&1 &
  sleep 2
  out=$(live_survivors)
  rc_live=$(( $(printf '%s' "$out" | grep -c . || true) - base ))
  say "-- A: LIVE process -- live_survivors delta = $rc_live (want >=1)"
  pkill -f nav2_costmap_2d_livectl >/dev/null 2>&1
  sleep 2

  # ---- B: a genuine ZOMBIE must NOT be refused ----
  # ⚑ MAKING A REAL ZOMBIE IS FIDDLIER THAN IT LOOKS, AND TWO EARLIER ATTEMPTS PROVED
  # NOTHING WHILE APPEARING TO.
  #   (i)  `exec -a NAME ...` sets argv[0] but NOT comm, and a zombie has an empty
  #        cmdline -- `ps` then shows the executable's comm, so the fake never carried
  #        the name at all and zero zombies were created. The control reported a clean
  #        negative half that had never been exercised.
  #   (ii) the spawning shell's OWN command line contained the pattern, so the predicate
  #        matched the live parent and the "zombie was not refused" half failed for a
  #        reason that had nothing to do with zombies.
  # Both are the same shape: a control that cannot fail for the reason it claims. This
  # version copies a real binary to a matching NAME (so comm matches, as `map_server`
  # did on the runner) and builds that name from fragments so the literal never appears
  # in the parent's argv.
  local zdir; zdir=$(mktemp -d)
  cat > "$zdir/mkzombie.py" <<'PYZ'
import os, sys, time, shutil
d = sys.argv[1]
name = "map_" + "server"            # never appears whole in this process's argv
path = os.path.join(d, name)
shutil.copy("/bin/true", path); os.chmod(path, 0o755)
if os.fork() == 0:
    os.execv(path, [path])          # child exits immediately
time.sleep(20)                      # parent never wait()s -> the child stays a zombie
PYZ
  setsid python3 "$zdir/mkzombie.py" "$zdir" >/dev/null 2>&1 &
  sleep 3
  zcount=$(ps -eo stat=,comm= | awk '$1 ~ /Z/ && $2 == "map_server"' | grep -c . || true)
  out=$(live_survivors)
  rc_zomb=$(( $(printf '%s' "$out" | grep -c . || true) - base ))
  say "-- B: ZOMBIE  -- zombies actually present = $zcount (want >=1); live_survivors delta = $rc_zomb (want 0)"
  pkill -f "$zdir" >/dev/null 2>&1; rm -rf "$zdir"; sleep 1

  if [ "$rc_live" -ge 1 ] && [ "$zcount" -ge 1 ] && [ "$rc_zomb" -eq 0 ]; then
    say "     BOTH CONTROLS PASS: a live survivor refuses, a real zombie does not."
    return 0
  fi
  say "     ⚑ CONTROL FAILED. live-delta=$rc_live (want >=1)  zombies-present=$zcount (want >=1)  zombie-delta=$rc_zomb (want 0)"
  [ "$zcount" -eq 0 ] && say "        No zombie existed, so the negative half proved NOTHING. It did not pass -- it was never tested."
  return 1
}

if [ "${2:-}" = "--selftest-contamination" ]; then
  selftest_contamination; exit $?
fi

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

DOMAIN=41
assert_clean(){
  local lbl="$1" surv n
  surv=$(live_survivors)
  n=$(printf '%s' "$surv" | grep -c . )
  if [ "$n" -ne 0 ]; then
    say "⚑ CONTAMINATED before case '$lbl': $n LIVE process(es) survived the previous case."
    say "   Zombies are excluded -- these are running and can join the ROS graph."
    say "   This gate will not report a number it cannot trust. Survivors:"
    printf '%s\n' "$surv" | sed 's/^/     /' | tee -a "$R"
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
