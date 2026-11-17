#!/usr/bin/env bash
set -euo pipefail
SIM_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
SIM_IMAGE=teb-nav-demo:humble
SIM_CONTAINER=teb-nav-demo
cd "$SIM_ROOT"
require_container() {
  if [[ $(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null || true) != true ]]; then
    echo "仿真容器未运行。先在一个终端执行 ./scripts/sim.sh gui（保持运行），再在另一个终端执行本命令。" >&2
    exit 1
  fi
}
case "${1:-help}" in
  restart)
    shift
    SIM_LAUNCH=${1:-gui}
    if [[ $# -gt 0 ]]; then shift; fi
    if [[ $SIM_LAUNCH != gui && $SIM_LAUNCH != start ]]; then
      echo 'Usage: ./scripts/sim.sh restart [gui|start] [launch args]' >&2
      exit 2
    fi
    if [[ $SIM_LAUNCH == gui ]]; then
      : "${DISPLAY:?GUI requires a local X11/XWayland display}"
    fi
    if docker container inspect "$SIM_CONTAINER" >/dev/null 2>&1; then
      echo "重建仿真：停止旧场景，重置小车、动态障碍和在线地图（保留 results）。"
      docker stop -t 15 "$SIM_CONTAINER" >/dev/null
      # Normally --rm already removes it; also handle a stopped legacy container.
      if docker container inspect "$SIM_CONTAINER" >/dev/null 2>&1; then
        docker rm "$SIM_CONTAINER" >/dev/null
      fi
    fi
    exec "$SIM_ROOT/scripts/sim.sh" "$SIM_LAUNCH" "$@"
    ;;
  image)
    docker build --build-arg SIM_UID="$(id -u)" --build-arg SIM_GID="$(id -g)" -t "$SIM_IMAGE" -f docker/Dockerfile .
    ;;
  build)
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e MAKEFLAGS=-j2 -e CMAKE_BUILD_PARALLEL_LEVEL=2 -v "$SIM_ROOT:/workspace" "$SIM_IMAGE" bash -c \
      'source /opt/ros/humble/setup.bash; cd /workspace; colcon build --symlink-install --packages-up-to dynamic_nav_sim --parallel-workers 2 --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release '
    ;;
  start)
    shift
    docker run --rm -d --stop-signal SIGINT --name "$SIM_CONTAINER" --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -v "$SIM_ROOT:/workspace" "$SIM_IMAGE" bash -c \
      'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; exec ros2 launch dynamic_nav_sim dynamic_avoidance.launch.py "$@"' bash "$@"
    ;;
  gui)
    shift
    : "${DISPLAY:?GUI requires a local X11/XWayland display}"
    SIM_XAUTH=$(mktemp)
    trap 'rm -f "$SIM_XAUTH"' EXIT
    xauth nlist "$DISPLAY" | sed 's/^..../ffff/' | xauth -f "$SIM_XAUTH" nmerge -
    docker run --rm --stop-signal SIGINT --name "$SIM_CONTAINER" --user "$(id -u):$(id -g)" -e HOME=/tmp \
      -e DISPLAY -e XAUTHORITY=/tmp/sim.xauth -e LIBGL_ALWAYS_SOFTWARE=1 -e LP_NUM_THREADS="${SIM_RENDER_THREADS:-2}" \
      -v "$SIM_XAUTH:/tmp/sim.xauth:ro" -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v "$SIM_ROOT:/workspace" "$SIM_IMAGE" bash -c \
      'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; exec ros2 launch dynamic_nav_sim dynamic_avoidance.launch.py gui:=true rviz:=true "$@"' bash "$@"
    ;;
  trial|status)
    SIM_COMMAND=$1
    shift
    require_container
    if [[ $SIM_COMMAND == status ]]; then
      set -- --check-only --ready-timeout 10 "$@"
    fi
    echo "进入容器 $SIM_CONTAINER 检查仿真通信……"
    docker exec "$SIM_CONTAINER" bash -c \
      'source /opt/ros/humble/setup.bash; source /workspace/install/setup.bash; exec ros2 run dynamic_nav_sim run_trial.py "$@"' bash "$@"
    ;;
  resume)
    require_container
    docker exec "$SIM_CONTAINER" bash -c \
      'source /opt/ros/humble/setup.bash; exec timeout 15 ros2 service call /unpause_physics std_srvs/srv/Empty "{}"'
    echo "已请求恢复仿真。执行 ./scripts/sim.sh status 检查时钟和传感器，再运行 trial。"
    ;;
  corridor)
    shift
    exec "$SIM_ROOT/scripts/corridor.sh" "$@"
    ;;
  shell)
    docker exec -it "$SIM_CONTAINER" bash --rcfile /workspace/scripts/container.bashrc
    ;;
  logs) docker logs -f "$SIM_CONTAINER" ;;
  stop) docker stop -t 15 "$SIM_CONTAINER" ;;
  *)
    echo "Usage: $0 {image|build|start [launch args]|gui [launch args]|restart [gui|start] [launch args]|status|resume|trial [--mode teb|straight] [--ready-timeout seconds] [--verbose-report]|corridor [--gif]|shell|logs|stop}"
    ;;
esac
