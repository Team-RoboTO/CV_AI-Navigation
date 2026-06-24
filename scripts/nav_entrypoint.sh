#!/usr/bin/env bash
set -e

echo "[nav_entrypoint] START"
date

# NOTA: NON sourciamo piu' /root/.bashrc per le variabili critiche.
# .bashrc ha la guardia "[ -z "$PS1" ] && return": quando questo script
# viene eseguito via `docker exec` SENZA tty (es. da un service systemd),
# $PS1 e' vuoto, .bashrc fa return immediato e tutte le export DDS/ROS
# che stanno sotto (incluso FASTRTPS_DEFAULT_PROFILES_FILE) non vengono
# mai applicate. Per questo "a mano" (docker exec -it) funzionava e da
# service no. Le esportiamo qui esplicitamente, sempre, in ogni caso.

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

# --- Variabili ROS/DDS, esplicite e non condizionate da .bashrc ---
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# QUESTO e' il pezzo che mancava in modo silenzioso via service:
# forza FastDDS a non usare la shared memory transport, indispensabile
# per la discovery cross-container senza --ipc=host condiviso.
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-/root/dds_config/fastdds_no_shm.xml}"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTDDS_DEFAULT_PROFILES_FILE:-/root/dds_config/fastdds_no_shm.xml}"

# Verifica esplicita che il file esista davvero: se manca, meglio fallire
# subito con un errore chiaro che scoprirlo dopo con la navigazione rotta.
if [ ! -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]; then
  echo "[nav_entrypoint] ERROR: FASTRTPS_DEFAULT_PROFILES_FILE non trovato: ${FASTRTPS_DEFAULT_PROFILES_FILE}"
  exit 1
fi

echo "[nav_entrypoint] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "[nav_entrypoint] RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
echo "[nav_entrypoint] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-unset}"
echo "[nav_entrypoint] CYCLONEDDS_URI=${CYCLONEDDS_URI:-unset}"
echo "[nav_entrypoint] FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset}"
echo "[nav_entrypoint] FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE:-unset}"

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

export NAV_SUPERVISOR_CLEANUP="${NAV_SUPERVISOR_CLEANUP:-1}"

export COMPETITION_CMD="source /opt/ros/humble/setup.bash && source ${WORKSPACE}/install/setup.bash && export ROS_DOMAIN_ID=0 && export RMW_IMPLEMENTATION=rmw_fastrtps_cpp && export ROS_LOCALHOST_ONLY=0 && export FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE} && export FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE} && echo [competition_cmd] ROS_DOMAIN_ID=\$ROS_DOMAIN_ID && echo [competition_cmd] RMW_IMPLEMENTATION=\$RMW_IMPLEMENTATION && echo [competition_cmd] ROS_LOCALHOST_ONLY=\$ROS_LOCALHOST_ONLY && echo [competition_cmd] FASTRTPS_DEFAULT_PROFILES_FILE=\$FASTRTPS_DEFAULT_PROFILES_FILE && ros2 launch nav2_new competition_match.launch.py team:=red waypoints_file:=${WORKSPACE}/src/nav2_new/config/arena_waypoints_lab.yaml match_params_file:=${WORKSPACE}/src/nav2_new/config/match_manager_params_lab.yaml map:=${WORKSPACE}/maps/lab_prova5.yaml"

echo "[nav_entrypoint] STARTUP_GRACE_SEC=${STARTUP_GRACE_SEC}"
echo "[nav_entrypoint] OUT_OF_BOUNDS_LIMIT=${OUT_OF_BOUNDS_LIMIT}"
echo "[nav_entrypoint] NO_EFFECTIVE_LIMIT=${NO_EFFECTIVE_LIMIT}"
echo "[nav_entrypoint] MISSING_MAP_TF_LIMIT=${MISSING_MAP_TF_LIMIT}"
echo "[nav_entrypoint] WINDOW_SEC=${WINDOW_SEC}"
echo "[nav_entrypoint] RESTART_DELAY_SEC=${RESTART_DELAY_SEC}"
echo "[nav_entrypoint] MAX_RESTART_DELAY_SEC=${MAX_RESTART_DELAY_SEC}"
echo "[nav_entrypoint] NAV_SUPERVISOR_CLEANUP=${NAV_SUPERVISOR_CLEANUP}"
echo "[nav_entrypoint] COMPETITION_CMD=${COMPETITION_CMD}"

echo "[nav_entrypoint] cleaning stale nav processes before start..."

pkill -INT -f "competition_supervisor.py" || true
pkill -INT -f "competition_match.launch.py" || true
pkill -INT -f "game_state_manager" || true
pkill -INT -f "waypoint_manager" || true
pkill -INT -f "micro_status_adapter" || true

sleep 1

pkill -TERM -f "competition_supervisor.py" || true
pkill -TERM -f "competition_match.launch.py" || true
pkill -TERM -f "game_state_manager" || true
pkill -TERM -f "waypoint_manager" || true
pkill -TERM -f "micro_status_adapter" || true

sleep 1

echo "[nav_entrypoint] stale process cleanup done"

echo "[nav_entrypoint] restarting local ros2 daemon..."
ros2 daemon stop || true
ros2 daemon start || true
echo "[nav_entrypoint] ros2 daemon restarted"

echo "[nav_entrypoint] launching supervisor..."
exec python3 -u "${SUPERVISOR_PATH}"