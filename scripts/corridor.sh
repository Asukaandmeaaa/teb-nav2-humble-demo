#!/usr/bin/env bash
set -euo pipefail
SIM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_CONTAINER=teb-nav-demo
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --gif ) ]]; then
  echo 'Usage: ./scripts/sim.sh corridor [--gif]' >&2
  exit 2
fi
if docker inspect "$SIM_CONTAINER" >/dev/null 2>&1; then
  echo 'Stop the current simulation with ./scripts/sim.sh stop before running a fresh corridor trial.' >&2
  exit 1
fi
python3 -c 'import numpy, matplotlib; from PIL import Image'
SIM_RUN=corridor_$(date -u +%Y%m%d_%H%M%S_UTC)
SIM_OUT="$SIM_ROOT/results/$SIM_RUN"
mkdir -p "$SIM_OUT"
SIM_PACKAGE="$SIM_ROOT/src/dynamic_nav_sim"
cp "$SIM_PACKAGE/config/long_corridor_scene.json" "$SIM_OUT/scene_used.json"
cp "$SIM_PACKAGE/config/nav2_responsive_doors.yaml" "$SIM_OUT/nav2_used.yaml"
cp "$SIM_PACKAGE/config/slam_online.yaml" "$SIM_OUT/slam_used.yaml"
cp "$SIM_PACKAGE/config/generic_sensors.json" "$SIM_OUT/sensors_used.json"
cp "$SIM_PACKAGE/config/gazebo.yaml" "$SIM_OUT/gazebo_used.yaml"
cp "$SIM_PACKAGE/launch/dynamic_avoidance.launch.py" "$SIM_OUT/launch_used.py"
SIM_OWNED=0
cleanup() {
  if [[ "$SIM_OWNED" == 1 ]]; then
    docker logs "$SIM_CONTAINER" > "$SIM_OUT/launch.log" 2>&1 || true
    "$SIM_ROOT/scripts/sim.sh" stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
"$SIM_ROOT/scripts/sim.sh" start scene:=long_corridor profile:=responsive > "$SIM_OUT/container.txt"
SIM_OWNED=1
SIM_EXIT=0
"$SIM_ROOT/scripts/sim.sh" trial --output "/workspace/results/$SIM_RUN" > "$SIM_OUT/console.log" 2>&1 || SIM_EXIT=$?
docker logs "$SIM_CONTAINER" > "$SIM_OUT/launch.log" 2>&1
"$SIM_ROOT/scripts/sim.sh" stop >/dev/null
SIM_OWNED=0
SIM_REPORTS=("$SIM_OUT"/trial_*.json)
if [[ -f "${SIM_REPORTS[0]}" ]]; then
  python3 "$SIM_ROOT/scripts/plot_corridor.py" --trial "${SIM_REPORTS[0]}" --output "$SIM_OUT" "$@"
fi
cat "$SIM_OUT/console.log"
echo "Results: $SIM_OUT"
exit "$SIM_EXIT"
