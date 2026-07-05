#!/usr/bin/env bash
# kill_all_node.sh — Detect running ROS2 nodes and stop them (plus ROS2 tools/daemon).
#
# Usage:
#   ./kill_all_node.sh            # graceful SIGINT, then SIGKILL leftovers
#   ./kill_all_node.sh -9         # force SIGKILL immediately (no graceful wait)
#   ./kill_all_node.sh -l         # list active nodes only, do NOT kill (dry run)
#
# Strategy (ports ~/.../kill_all_ros2.sh, kept as reference SSOT):
#   1) `ros2 node list` to discover live nodes (detection).
#   2) Match each node to its PID via `pgrep -f __node:=<name>` and signal it.
#   3) Sweep ROS2 tool processes (rviz2/rqt/rosbag2/launch) as a fallback.
#   4) Stop the ROS2 daemon, then verify (retry once with SIGKILL if needed).
#
# NOTE: this stops ALL ROS2 activity for the current ROS_DOMAIN_ID, including
# nodes started from other terminals/workspaces on the same domain.

set -u

FORCE=0        # -9 : skip graceful stage
LIST_ONLY=0    # -l : dry run

for arg in "$@"; do
  case "$arg" in
    -9|--force) FORCE=1 ;;
    -l|--list)  LIST_ONLY=1 ;;
    -h|--help)  sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "[kill_all_node] unknown option: $arg" >&2; exit 2 ;;
  esac
done

# --- detection --------------------------------------------------------------
list_nodes() { ros2 node list 2>/dev/null | sed '/^[[:space:]]*$/d'; }

NODES="$(list_nodes)"

echo "=== ROS2 node detection ==="
if [ -z "$NODES" ]; then
  echo "No active ROS2 nodes found."
else
  echo "$NODES"
fi

if [ "$LIST_ONLY" -eq 1 ]; then
  exit 0
fi

# --- helper: signal every PID that backs a given node -----------------------
# Try the precise remap match first (__node:=<short>), fall back to the name.
kill_node() {
  local node="$1" sig="$2"
  local short="${node##*/}"          # /ns/foo -> foo
  local pids
  pids="$(pgrep -f -- "__node:=${short}" 2>/dev/null)"
  [ -z "$pids" ] && pids="$(pgrep -f -- "$short" 2>/dev/null)"
  for pid in $pids; do
    kill "-$sig" "$pid" 2>/dev/null && echo "  ${sig}  pid=$pid  node=$node"
  done
}

# --- graceful stage (SIGINT) ------------------------------------------------
if [ "$FORCE" -eq 0 ] && [ -n "$NODES" ]; then
  echo "=== Stopping nodes (SIGINT) ==="
  for node in $NODES; do kill_node "$node" INT; done
  sleep 2
  NODES="$(list_nodes)"
fi

# --- force stage (SIGKILL) --------------------------------------------------
if [ -n "$NODES" ]; then
  echo "=== Stopping remaining nodes (SIGKILL) ==="
  for node in $NODES; do kill_node "$node" KILL; done
fi

# --- fallback sweep of ROS2 tools -------------------------------------------
echo "=== Sweeping ROS2 tools ==="
for pat in "ros2 launch" "ros2 run" rviz2 rqt rosbag2 component_container; do
  pkill -9 -f -- "$pat" 2>/dev/null && echo "  killed: $pat"
done

# --- stop the daemon --------------------------------------------------------
ros2 daemon stop >/dev/null 2>&1
pkill -9 -f -- "ros2-daemon" 2>/dev/null
sleep 1

# --- verify -----------------------------------------------------------------
echo "=== Verification ==="
REMAINING="$(list_nodes)"
if [ -z "$REMAINING" ]; then
  echo "OK: no ROS2 nodes remaining."
  exit 0
fi

echo "Still alive, forcing kill:"
echo "$REMAINING"
for node in $REMAINING; do kill_node "$node" KILL; done
sleep 1

FINAL="$(list_nodes)"
if [ -z "$FINAL" ]; then
  echo "OK: all ROS2 nodes killed."
  exit 0
fi
echo "WARNING: still remaining (may be on another host/domain):" >&2
echo "$FINAL" >&2
exit 1
