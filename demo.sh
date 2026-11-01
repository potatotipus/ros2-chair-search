#!/usr/bin/env bash
# Usage: bash demo.sh prepare|sim|nav|undock|ai|camera|rviz|search|check|status
set -e -o pipefail
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT MESA_LOADER_DRIVER_OVERRIDE
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export PYTHONPATH="$DEMO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$DEMO_ROOT/runtime/logs"
cd "$DEMO_ROOT"
action="${1:-help}"
case "$action" in
  prepare)
    shift
    /usr/bin/python3 "$DEMO_ROOT/prepare_demo.py" "$@"
    ;;
  sim)
    ros2 launch "$DEMO_ROOT/room_demo.launch.py" 2>&1 | tee "$DEMO_ROOT/runtime/logs/sim.log"
    ;;
  nav)
    ros2 launch "$DEMO_ROOT/room_navigation.launch.py" 2>&1 | tee "$DEMO_ROOT/runtime/logs/nav.log"
    ;;
  undock)
    ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
    ;;
  ai)
    detector_python="$DEMO_ROOT/.venv/bin/python"
    if [[ ! -x "$detector_python" && -x "$HOME/robot_ai/.venv/bin/python" ]]; then
      detector_python="$HOME/robot_ai/.venv/bin/python"
    fi
    if [[ ! -x "$detector_python" ]]; then
      echo 'AI environment missing. Follow docs/SETUP.md to create .venv.' >&2
      exit 1
    fi
    if [[ ! -f "$DEMO_ROOT/yolo11s.pt" && -f "$HOME/robot_ai/yolo11s.pt" ]]; then
      ln -s "$HOME/robot_ai/yolo11s.pt" "$DEMO_ROOT/yolo11s.pt"
    fi
    "$detector_python" "$DEMO_ROOT/camera_detector.py" --ros-args -p use_sim_time:=true 2>&1 | tee "$DEMO_ROOT/runtime/logs/ai.log"
    ;;
  camera)
    ros2 run rqt_image_view rqt_image_view
    ;;
  rviz)
    rviz2 -d "$DEMO_ROOT/room.rviz" --ros-args -p use_sim_time:=true
    ;;
  search)
    read -r demo_x demo_y < <(/usr/bin/python3 -c 'from room_geometry import OBSERVATION; print(*OBSERVATION)')
    /usr/bin/python3 "$DEMO_ROOT/room_search.py" --ros-args -p use_sim_time:=true -p "x:=$demo_x" -p "y:=$demo_y" 2>&1 | tee "$DEMO_ROOT/runtime/logs/search.log"
    ;;
  check)
    /usr/bin/python3 "$DEMO_ROOT/check_scan.py" --ros-args -p use_sim_time:=true
    ros2 lifecycle get /amcl
    ros2 lifecycle get /bt_navigator
    ros2 action info /navigate_to_pose
    ;;
  status)
    free -h
    ps -eo pid,comm,rss,args --sort=-rss | head -n 10 || true
    ;;
  *)
    echo 'Usage: bash demo.sh prepare|sim|nav|undock|ai|camera|rviz|search|check|status'
    ;;
esac
