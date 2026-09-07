#!/usr/bin/env bash
# ══════════════════════════════════════════════════════════════════════════════
# hubot_live_stack — build gate.
#
# ⚑ WHY THIS FILE EXISTS, AND IT IS NOT "run colcon and check the exit code".
#
# This package lives INSIDE the `hubot` package's own directory, and `hubot`'s
# repository root carries a package.xml. colcon's recursive crawl STOPS at the
# first package.xml on a path, so from a normal workspace this package is
# INVISIBLE. Measured 2026-09-07 on ubuntu 26.04 + colcon-core 0.21.1, workspace
# `src/hubot` = a clone of this repository:
#
#     $ colcon list --names-only
#     hubot                              <- and nothing else
#
#     $ colcon build --packages-select hubot_live_stack
#     WARNING:colcon.colcon_core.package_selection:
#         ignoring unknown package 'hubot_live_stack' in --packages-select
#     Summary: 0 packages finished [0.07s]
#     rc=0
#
# ⚑ THAT LAST LINE IS THE WHOLE REASON FOR THIS SCRIPT. Asking colcon to build
# this package builds NOTHING and exits ZERO. A green build that built nothing is
# indistinguishable from a green build that built everything -- which is PI-9 of
# `doc/SOTIF_PERFORMANCE_INSUFFICIENCY.md` ("loaded, configured, activated and
# driven ZERO times inside a stack that reports itself healthy") arriving in the
# build system instead of the costmap.
#
# The same crawl rule applies to rosdep: `rosdep install --from-paths src/hubot`
# resolves 17 keys and does not see this package's three extra ones. And rosdep
# pointed at THIS directory alone fails outright --
#     ERROR: hubot_live_stack: Cannot locate rosdep definition for [hubot]
# -- because `hubot` is an <exec_depend> here and is not a published rosdep key.
# Both instruments must therefore be given BOTH paths explicitly.
#
# So: build by explicit path, then ASSERT THE ARTIFACTS EXIST. The exit code is
# not the evidence; the install space is.
#
# Usage:  bash build_gate.sh <workspace>            # build + assert
#         bash build_gate.sh <workspace> --selftest # assert the guard can FAIL
# ══════════════════════════════════════════════════════════════════════════════
set -uo pipefail

WS="${1:?usage: build_gate.sh <workspace> [--selftest]}"
MODE="${2:-}"
SRC="$WS/src/hubot"
HLS="$SRC/hubot_live_stack"

[ -f "$SRC/package.xml" ] || { echo "no hubot package at $SRC" >&2; exit 64; }
[ -f "$HLS/package.xml" ] || { echo "no hubot_live_stack package at $HLS" >&2; exit 64; }

# The four executables and the one plugin library this package promises. Named
# here rather than globbed, so deleting a target from CMakeLists.txt turns this
# red instead of silently shrinking what "built" means.
WANT_BINS="param_target_node scenario_support_node bt_clear_costmap_driver decision_observer"
WANT_LIB="libhubot_live_stack_layers.so"
WANT_SHARE="config/live_stack.yaml scripts/run_live_stack.sh hubot_live_stack_plugins.xml"

assert_install_space() {   # assert_install_space <install-base> ; 0 = complete
  local base="$1" missing=0 p
  for b in $WANT_BINS; do
    p="$base/hubot_live_stack/lib/hubot_live_stack/$b"
    [ -x "$p" ] || { echo "MISSING executable: $p"; missing=1; }
  done
  p="$base/hubot_live_stack/lib/$WANT_LIB"
  [ -f "$p" ] || { echo "MISSING plugin library: $p"; missing=1; }
  for s in $WANT_SHARE; do
    p="$base/hubot_live_stack/share/hubot_live_stack/$s"
    [ -e "$p" ] || { echo "MISSING shipped file: $p"; missing=1; }
  done
  # hubot itself must still be there: this package is worthless without the
  # filter it exists to drive.
  [ -f "$base/hubot/lib/libhubot_costmap_filters.so" ] || {
    echo "MISSING hubot filter library: $base/hubot/lib/libhubot_costmap_filters.so"; missing=1; }
  return "$missing"
}

if [ "$MODE" = "--selftest" ]; then
  # ⚑ THE GUARD MUST BE ABLE TO REFUSE, and the way it fails in the field is the
  # silent one above: a build that selects a package colcon never discovered.
  # Run exactly that, and require the guard to say NO. If this passes, the guard
  # below is blind and its green means nothing.
  echo "-- selftest: the invocation that silently builds nothing"
  cd "$WS" || exit 70
  rm -rf "$WS/install_selftest" "$WS/build_selftest"
  colcon build --packages-select hubot_live_stack \
    --install-base "$WS/install_selftest" --build-base "$WS/build_selftest" 2>&1 | tail -3
  rc=${PIPESTATUS[0]}
  echo "   colcon exit code was: $rc   <- note this is the value a naive gate would trust"
  if assert_install_space "$WS/install_selftest" >/dev/null 2>&1; then
    echo "SELFTEST FAILED: the guard accepted an install space that was never built."
    exit 1
  fi
  echo "SELFTEST PASSED: colcon exited $rc and the guard still refused."
  exit 0
fi

echo "-- building hubot + hubot_live_stack by EXPLICIT path (the crawl will not find the second)"
cd "$WS" || exit 70
colcon build --paths "$SRC" "$HLS" --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -8
rc=${PIPESTATUS[0]}
if [ "$rc" -ne 0 ]; then
  echo "BUILD FAILED (colcon exit $rc)"
  exit "$rc"
fi

echo "-- asserting the install space actually carries what this package promises"
if ! assert_install_space "$WS/install"; then
  echo "GATE RED: colcon exited 0 and the artifacts are not there."
  exit 1
fi
echo "GATE GREEN: hubot + hubot_live_stack built, and every promised artifact is present."
