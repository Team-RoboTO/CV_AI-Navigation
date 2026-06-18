#!/usr/bin/env python3

import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime

print("[competition_supervisor] STARTED", flush=True)

CMD = os.environ.get(
    "COMPETITION_CMD",
    "source /opt/ros/humble/setup.bash && "
    "source /root/nav2_ws/install/setup.bash && "
    "ros2 launch nav2_new competition_match.launch.py "
    "team:=red "
    "waypoints_file:=/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml "
    "match_params_file:=/root/nav2_ws/src/nav2_new/config/match_manager_params_lab.yaml"
)

STARTUP_GRACE_SEC = float(os.environ.get("STARTUP_GRACE_SEC", "12"))
RESTART_DELAY_SEC = float(os.environ.get("RESTART_DELAY_SEC", "3"))

BAD_PATTERNS = [
    re.compile(r"No Effective Points!"),

    # Pose / TF / localization fuori scala
    re.compile(r"Robot is out of bounds of the costmap"),
    re.compile(r"Sensor origin .* is out of map bounds"),
    re.compile(r"The costmap cannot raytrace for it"),

    # Planner/costmap fatal
    re.compile(r"Cannot create a plan: the robot's start position is off the global costmap"),
    re.compile(r"worldToMap failed"),
    re.compile(r"Pose Goes Off Grid"),

    # TF odom-base rotto
    re.compile(r"Timed out waiting for transform from base_link to odom"),
    re.compile(r"Could not find a connection between 'odom' and 'base_link'"),
    re.compile(r"two or more unconnected trees"),
]

STOP = False
PROC = None


def ts():
    return datetime.now().strftime("%H:%M:%S")


def on_signal(signum, frame):
    global STOP
    STOP = True
    print(f"\n[{ts()}] supervisor: stop requested", flush=True)
    if PROC is not None and PROC.poll() is None:
        try:
            PROC.send_signal(signal.SIGINT)
        except Exception:
            pass


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


def stop_proc(proc):
    if proc.poll() is not None:
        return

    print(f"[{ts()}] supervisor: stopping launch", flush=True)

    try:
        proc.send_signal(signal.SIGINT)
    except Exception:
        pass

    for _ in range(80):
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    try:
        proc.terminate()
    except Exception:
        pass

    for _ in range(30):
        if proc.poll() is not None:
            return
        time.sleep(0.1)

    try:
        proc.kill()
    except Exception:
        pass


def run_once(index):
    global PROC

    print("", flush=True)
    print(f"[{ts()}] supervisor: launch attempt {index}", flush=True)
    print(f"[{ts()}] supervisor: CMD={CMD}", flush=True)

    PROC = subprocess.Popen(
        ["bash", "-lc", CMD],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    start = time.monotonic()
    bad_count = 0
    last_heartbeat = start

    assert PROC.stdout is not None

    while not STOP:
        line = PROC.stdout.readline()
        now = time.monotonic()

        if now - last_heartbeat > 5.0:
            print(f"[{ts()}] supervisor: alive, pid={PROC.pid}", flush=True)
            last_heartbeat = now

        if line == "":
            if PROC.poll() is not None:
                print(f"[{ts()}] supervisor: launch exited code {PROC.returncode}", flush=True)
                return
            time.sleep(0.05)
            continue

        print(line, end="", flush=True)

        if now - start < STARTUP_GRACE_SEC:
            continue

        if any(p.search(line) for p in BAD_PATTERNS):
            bad_count += 1
            print(f"[{ts()}] supervisor: bad startup counter {bad_count}/12", flush=True)

        if bad_count >= 12:
            print(f"[{ts()}] supervisor: bad startup detected, restarting", flush=True)
            stop_proc(PROC)
            return

    stop_proc(PROC)


def main():
    i = 1

    while not STOP:
        run_once(i)
        i += 1

        if STOP:
            break

        print(f"[{ts()}] supervisor: restart in {RESTART_DELAY_SEC}s", flush=True)
        time.sleep(RESTART_DELAY_SEC)

    print(f"[{ts()}] supervisor: stopped", flush=True)


if __name__ == "__main__":
    main()
