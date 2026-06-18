#!/usr/bin/env bash
set -e

echo "[nav_entrypoint] START"
date

WORKSPACE="${WORKSPACE:-/root/nav2_ws}"
echo "[nav_entrypoint] WORKSPACE=${WORKSPACE}"

echo "[nav_entrypoint] checking files..."
ls -ld "${WORKSPACE}" || exit 1
ls -l /opt/ros/humble/setup.bash || exit 1
ls -l "${WORKSPACE}/install/setup.bash" || exit 1

echo "[nav_entrypoint] sourcing /opt/ros/humble/setup.bash..."
source /opt/ros/humble/setup.bash
echo "[nav_entrypoint] sourced ROS Humble"

echo "[nav_entrypoint] sourcing ${WORKSPACE}/install/setup.bash..."
source "${WORKSPACE}/install/setup.bash"
echo "[nav_entrypoint] sourced workspace"

export PYTHONUNBUFFERED=1

SUPERVISOR_PATH="${NAV_SUPERVISOR_PATH:-${WORKSPACE}/scripts/competition_supervisor.py}"

if [ ! -f "${SUPERVISOR_PATH}" ]; then
  ALT_SUPERVISOR_PATH="${WORKSPACE}/src/nav2_new/scripts/competition_supervisor.py"

  if [ -f "${ALT_SUPERVISOR_PATH}" ]; then
    SUPERVISOR_PATH="${ALT_SUPERVISOR_PATH}"
  else
    echo "[nav_entrypoint] ERROR: competition_supervisor.py not found."
    echo "[nav_entrypoint] Tried:"
    echo "  ${WORKSPACE}/scripts/competition_supervisor.py"
    echo "  ${ALT_SUPERVISOR_PATH}"
    exit 1
  fi
fi

echo "[nav_entrypoint] SUPERVISOR_PATH=${SUPERVISOR_PATH}"
ls -l "${SUPERVISOR_PATH}" || exit 1

export STARTUP_GRACE_SEC="${STARTUP_GRACE_SEC:-12}"
export OUT_OF_BOUNDS_LIMIT="${OUT_OF_BOUNDS_LIMIT:-3}"
export NO_EFFECTIVE_LIMIT="${NO_EFFECTIVE_LIMIT:-60}"
export MISSING_MAP_TF_LIMIT="${MISSING_MAP_TF_LIMIT:-12}"
export WINDOW_SEC="${WINDOW_SEC:-8}"
export RESTART_DELAY_SEC="${RESTART_DELAY_SEC:-3}"
export MAX_RESTART_DELAY_SEC="${MAX_RESTART_DELAY_SEC:-30}"

# Durante i test manuali meglio 0.
# Quando funziona bene puoi metterlo a 1.
export NAV_SUPERVISOR_CLEANUP="${NAV_SUPERVISOR_CLEANUP:-0}"

export COMPETITION_CMD="${COMPETITION_CMD:-source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && ros2 launch nav2_new competition_match.launch.py team:=red waypoints_file:=${WORKSPACE}/src/nav2_new/config/arena_waypoints_lab.yaml match_params_file:=${WORKSPACE}/src/nav2_new/config/match_manager_params_lab.yaml map:=${WORKSPACE}/maps/lab_prova5.yaml}"

echo "[nav_entrypoint] STARTUP_GRACE_SEC=${STARTUP_GRACE_SEC}"
echo "[nav_entrypoint] OUT_OF_BOUNDS_LIMIT=${OUT_OF_BOUNDS_LIMIT}"
echo "[nav_entrypoint] NO_EFFECTIVE_LIMIT=${NO_EFFECTIVE_LIMIT}"
echo "[nav_entrypoint] MISSING_MAP_TF_LIMIT=${MISSING_MAP_TF_LIMIT}"
echo "[nav_entrypoint] NAV_SUPERVISOR_CLEANUP=${NAV_SUPERVISOR_CLEANUP}"
echo "[nav_entrypoint] COMPETITION_CMD=${COMPETITION_CMD}"

echo "[nav_entrypoint] launching supervisor..."
exec python3 -u "${SUPERVISOR_PATH}"