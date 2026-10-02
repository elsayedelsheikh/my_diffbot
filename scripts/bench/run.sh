#!/usr/bin/env bash
# Run a bench script in a sidecar container next to the bringup; data lands on the host in
# $KESTREL_BENCH_DATA (default ~/kestrel_bench), so it survives a Jetson reboot.
#   scripts/bench/run.sh bench <test> [k=v ...]      scripts/bench/run.sh analyze <run> ...
set -euo pipefail
DATA=${KESTREL_BENCH_DATA:-$HOME/kestrel_bench}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$DATA/out"
script=$1
shift
exec docker run --rm --network host --ipc host --user "$(id -u):$(id -g)" \
  -e HOME=/data -e ROS_LOG_DIR=/data/.roslog -e BENCH_DIR=/data \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -e FENCE="${FENCE:--0.85,0.35,-0.35,0.35}" -e PYTHONDONTWRITEBYTECODE=1 -e TIMEOUT="${TIMEOUT:-300}" \
  -v "$HERE:/bench:ro" -v "$DATA:/data" merlin:dev \
  bash -lc 'source /opt/ros/jazzy/setup.bash && cd /bench && timeout "$TIMEOUT" python3 "$0.py" "$@"' "$script" "$@"
