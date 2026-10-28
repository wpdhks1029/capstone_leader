#!/usr/bin/env bash
# Packaging helper. Never starts motors unless base is explicitly enabled.
set -e
LEADER_REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$LEADER_REPO_ROOT/install/setup.bash" ]]; then
  echo 'Build this checkout first: colcon build --symlink-install' >&2
  exit 1
fi
source "$LEADER_REPO_ROOT/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-10}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
case "${1:-help}" in
  base)
    LEADER_ALLOW_MOTION="${2:-false}"
    [[ "$LEADER_ALLOW_MOTION" == false || "$LEADER_ALLOW_MOTION" == true ]] || exit 2
    exec ros2 launch "$LEADER_REPO_ROOT/diagnostics/leader_nav_015_base.launch.py" "allow_motion:=$LEADER_ALLOW_MOTION"
    ;;
  localization|navigation)
    LEADER_RUNTIME_DIR="$(mktemp -d /tmp/capstone_leader_params.XXXXXX)"
    trap 'rm -f -- "$LEADER_RUNTIME_DIR/nav2.yaml"; rmdir -- "$LEADER_RUNTIME_DIR"' EXIT
    python3 "$LEADER_REPO_ROOT/scripts/render_nav2.py" "$LEADER_RUNTIME_DIR/nav2.yaml"
    if [[ "$1" == localization ]]; then
      ros2 launch nav2_bringup localization_launch.py \
        "map:=$LEADER_REPO_ROOT/maps/leader_map.yaml" \
        "params_file:=$LEADER_RUNTIME_DIR/nav2.yaml" use_sim_time:=false autostart:=true
    else
      ros2 launch nav2_bringup navigation_launch.py \
        "params_file:=$LEADER_RUNTIME_DIR/nav2.yaml" use_sim_time:=false autostart:=true
    fi
    ;;
  bridge)
    exec ros2 run convoy_robot real_leader_bridge_node.py --ros-args \
      --params-file "$LEADER_REPO_ROOT/src/convoy_robot/config/real_leader_bridge.yaml"
    ;;
  *)
    echo 'Usage: scripts/run_leader.sh {base [false|true]|localization|navigation|bridge}'
    ;;
esac
