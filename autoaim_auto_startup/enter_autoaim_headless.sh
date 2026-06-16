#!/usr/bin/env bash
set -euo pipefail

CONTAINER="autoaim_headless"

DISPLAY_ID="$(ls /tmp/.X11-unix/X* 2>/dev/null | head -n 1 | sed 's#/tmp/.X11-unix/X#:#')"
AUTH_FILE="$(ps aux | grep -E 'Xorg|Xwayland' | grep -v grep | sed -n 's/.* -auth \([^ ]*\).*/\1/p' | head -n 1 || true)"

if [ -z "$DISPLAY_ID" ]; then
  echo "ERROR: no X display found in /tmp/.X11-unix/"
  exit 1
fi

echo "[autoaim] using DISPLAY=$DISPLAY_ID"

if [ -n "$AUTH_FILE" ]; then
  echo "[autoaim] using XAUTHORITY=$AUTH_FILE"
  sudo env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +local:
  sudo env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +SI:localuser:root
  sudo env XAUTHORITY="$AUTH_FILE" DISPLAY="$DISPLAY_ID" xhost +local:root
else
  echo "[autoaim] WARNING: no XAUTHORITY found, trying xhost without it"
  DISPLAY="$DISPLAY_ID" xhost +local: || true
fi

docker start "$CONTAINER" >/dev/null 2>&1 || true

exec docker exec -it \
  -e DISPLAY="$DISPLAY_ID" \
  -e QT_QPA_PLATFORM=xcb \
  "$CONTAINER" bash
