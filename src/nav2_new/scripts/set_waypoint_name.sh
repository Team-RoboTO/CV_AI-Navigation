#!/usr/bin/env bash
# =============================================================================
# set_waypoint_name.sh — Set the name for the next click in waypoint_editor
# =============================================================================
# Usage:
#   ./set_waypoint_name.sh blue_tunnel_1
#   ./set_waypoint_name.sh center_ne
#   ./set_waypoint_name.sh           # clears name (next click auto-numbers)
# =============================================================================
NAME="${1:-}"
ros2 topic pub --once /waypoint_editor/next_name std_msgs/String "{data: '${NAME}'}"
