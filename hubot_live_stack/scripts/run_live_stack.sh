#!/usr/bin/env bash
# hubot_live_stack — run the live stack INSIDE the nav2 container.
#
# Five OS processes. The point of the harness is that they are five, not one:
#
#   1. param_target_node        the filter's parameter target      (item 3)
#   2. scenario_support_node    filter info + mask + continuous TF
#   3. decision_observer        reads zone_decision off the wire
#   4. controller_server        THE REAL nav2 BINARY               (items 1+2)
#        -> nav2::NodeThread                    controller_server.cpp:72
#        -> Costmap2DROS::on_activate spawns map_update_thread_
#                                               costmap_2d_ros.cpp:314
#        -> mapUpdateLoop -> updateMap -> LayeredCostmap::updateMap
#        -> OUR FILTER's updateBounds/updateCosts, driven by nav2's own thread
#   5. bt_clear_costmap_driver  a real BT ClearEntireCostmap        (item 4)
#        -> nav2_msgs/srv/ClearEntireCostmap
#        -> ClearCostmapService -> Costmap2DROS::resetLayers()  :719
#
# SUBSTRATE ONLY. This script starts things, records what happened, and exits 0
# if the stack came up. It asserts NOTHING about enforced/watching semantics.
#
# ⚑ "EXITS 0 IF THE STACK CAME UP" WAS A PROMISE THIS SCRIPT DID NOT KEEP, AND THE
# WAY IT BROKE IS THE ONE THAT HIDES. Until 2026-09-07 it exited 0 unconditionally:
# the last command was an `ls`. Measured that day on released nav2, all three
# conditions printed `activate rc=1` -- the costmap never activated, no filter was
# ever driven -- and the script still returned 0. A caller reading the exit code was
# told the stack came up. That is this package's own PI-9 ("configure and activate
# return 0, everything looks green, the filter has never executed") reproduced in
# the harness built to find it. STAGE_RC below is the fix: every stage's code is
# kept, all artifacts are still written, and the script exits non-zero at the end if
# any stage failed.
#
# Usage (inside the container, after sourcing the three setup.bash files):
#   bash run_live_stack.sh [--outdir DIR] [--hold-secs N]
#                          [--target-args "--freeze-after-ms 4000"]
#                          [--support-args "-p stop_tf_after_ms:=6000"]
#                          [--skip-recovery]

set -u
OUTDIR=/ws/live_stack_run
HOLD_SECS=12
TARGET_ARGS=""
SUPPORT_ARGS=""
SKIP_RECOVERY=0
PARAMS=""
COSTMAP_LOG_LEVEL="${COSTMAP_LOG_LEVEL:-info}"
OVERLAYS=()

while [ $# -gt 0 ]; do
  case "$1" in
    --outdir) OUTDIR="$2"; shift 2;;
    --hold-secs) HOLD_SECS="$2"; shift 2;;
    --target-args) TARGET_ARGS="$2"; shift 2;;
    --support-args) SUPPORT_ARGS="$2"; shift 2;;
    --params) PARAMS="$2"; shift 2;;
    # Repeatable. Each overlay is a params file applied AFTER the base, so a
    # condition is expressed as a two-key diff and the subject and its controls
    # cannot drift apart.
    --overlay) OVERLAYS+=("$2"); shift 2;;
    # ⚑ THE ONE PIECE OF EVIDENCE THIS HARNESS EXISTS FOR IS BEHIND A DEBUG LOGGER.
    # `Costmap2DROS::mapUpdateLoop` emits "Map update time: ..." at RCLCPP_DEBUG
    # (the string is in libnav2_costmap_2d_core.so; the macro is not INFO). At the
    # default level a released-nav2 run produces ZERO such lines, so the claim that
    # `map_update_thread_` is really driving the filter at the configured rate is
    # unobservable in the log -- present, and invisible. Raise the costmap node's
    # level to see it. Default stays `info` because a full-debug costmap log buries
    # everything else.
    --costmap-log-level) COSTMAP_LOG_LEVEL="$2"; shift 2;;
    --skip-recovery) SKIP_RECOVERY=1; shift;;
    *) echo "unknown arg: $1" >&2; exit 64;;
  esac
done

CFGDIR="$(ros2 pkg prefix hubot_live_stack)/share/hubot_live_stack/config"
if [ -z "$PARAMS" ]; then
  PARAMS="$CFGDIR/live_stack.yaml"
fi
# Build the --params-file argument vector. Later files override earlier ones.
PARAM_ARGS=(--params-file "$PARAMS")
for ov in "${OVERLAYS[@]:-}"; do
  [ -n "$ov" ] || continue
  case "$ov" in
    /*) ovp="$ov";;
    *)  ovp="$CFGDIR/$ov";;
  esac
  if [ ! -f "$ovp" ]; then echo "overlay not found: $ovp" >&2; exit 66; fi
  PARAM_ARGS+=(--params-file "$ovp")
  echo "overlay: $ovp"
done

mkdir -p "$OUTDIR"
rm -f "$OUTDIR"/*.log "$OUTDIR"/*.jsonl

# One domain per run keeps concurrent runs from discovering each other. With
# --network=none only `lo` exists, so this is intra-host discovery only.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export RCUTILS_LOGGING_BUFFERED_STREAM=0

PIDS=()
cleanup() {
  echo "--- tearing down ---"
  for p in "${PIDS[@]:-}"; do
    kill -INT "$p" 2>/dev/null || true
  done
  sleep 2
  for p in "${PIDS[@]:-}"; do
    kill -KILL "$p" 2>/dev/null || true
  done
}
trap cleanup EXIT

start() {  # start <logname> <cmd...>
  local name="$1"; shift
  echo "--- starting $name: $* "
  "$@" > "$OUTDIR/$name.log" 2>&1 &
  PIDS+=("$!")
}

echo "=== hubot_live_stack: params=$PARAMS outdir=$OUTDIR domain=$ROS_DOMAIN_ID ==="

# 1. THE PARAMETER TARGET -- its own process, its own executor, answering on
#    nobody's schedule but its own. This is the whole of item 3.
# shellcheck disable=SC2086
start param_target \
  ros2 run hubot_live_stack param_target_node --ros-args -r __node:=zpf_target_node -- $TARGET_ARGS

# 2. Costmap-side preconditions. Without the latched filter-info + mask the
#    filter never activates; without continuous TF, getRobotPose() fails and the
#    real map_update_thread_ drives the filter ZERO times.
# shellcheck disable=SC2086
start support ros2 run hubot_live_stack scenario_support_node --ros-args $SUPPORT_ARGS

# 3. The observer, subscribed BEFORE the filter exists so no transition is missed.
start observer ros2 run hubot_live_stack decision_observer --ros-args \
  -p out_path:="$OUTDIR/transcript.jsonl"

sleep 3

# 4. THE REAL nav2 controller_server.
start controller_server ros2 run nav2_controller controller_server --ros-args \
  --log-level "local_costmap.local_costmap:=$COSTMAP_LOG_LEVEL" "${PARAM_ARGS[@]}"

sleep 4

# Lifecycle, driven the way nav2_lifecycle_manager drives it: ChangeState
# services, from outside the process. on_activate is what spawns
# map_update_thread_ at costmap_2d_ros.cpp:314.
STAGE_RC=0
echo "--- configure ---"
ros2 lifecycle set /controller_server configure  > "$OUTDIR/lifecycle_configure.log" 2>&1
rc=$?; echo "configure rc=$rc"; [ "$rc" -eq 0 ] || STAGE_RC=1
sleep 2
echo "--- activate ---"
ros2 lifecycle set /controller_server activate    > "$OUTDIR/lifecycle_activate.log" 2>&1
rc=$?; echo "activate rc=$rc"; [ "$rc" -eq 0 ] || STAGE_RC=1

echo "--- holding ${HOLD_SECS}s with the real map_update_thread_ driving the filter ---"
sleep "$HOLD_SECS"

if [ "$SKIP_RECOVERY" -eq 0 ]; then
  echo "--- firing the REAL ClearEntireCostmap behaviour-tree recovery ---"
  ros2 run hubot_live_stack bt_clear_costmap_driver \
    --service /local_costmap/clear_entirely_local_costmap \
    > "$OUTDIR/bt_recovery.log" 2>&1
  rc=$?; echo "bt_recovery rc=$rc"; [ "$rc" -eq 0 ] || STAGE_RC=1
  echo "--- holding 6s after the recovery ---"
  sleep 6
fi

# Evidence that the topology was the intended one, captured from the live graph
# rather than asserted by this script.
{
  echo "### ros2 node list"
  ros2 node list 2>&1
  echo
  echo "### ros2 service list | clear"
  ros2 service list 2>&1 | grep -i clear
  echo
  echo "### ros2 param list /local_costmap/local_costmap | zone"
  ros2 param list /local_costmap/local_costmap 2>&1 | grep -i zone
  echo
  # Which set_parameters path the filter actually built a client for. This is
  # what distinguishes a target that refused from a target never addressed.
  echo "### costmap node service clients (set_parameters path)"
  ros2 node info /local_costmap/local_costmap 2>&1 | grep -i "set_parameters" || true
  echo
  echo "### services matching zpf_target"
  ros2 service list 2>&1 | grep zpf_target || true
} > "$OUTDIR/graph.log" 2>&1

echo "=== run complete; artifacts in $OUTDIR ==="
ls -la "$OUTDIR"

# ⚑ THE EXIT CODE IS PART OF THE OBSERVABLE. Artifacts are written either way, so a
# failed run is still fully readable -- but a caller is told, rather than having to
# grep this script's stdout for `rc=` lines and hope the pattern still matches.
if [ "$STAGE_RC" -ne 0 ]; then
  echo "=== A LIFECYCLE OR RECOVERY STAGE FAILED. Read $OUTDIR/controller_server.log ==="
  exit 1
fi
exit 0
