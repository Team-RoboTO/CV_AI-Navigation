#!/usr/bin/env python3
"""
Real-time tracker debug visualizer.

Subscribes to /tracker/info and /tracker/target and plots:
  1. Position innovation (position_diff) — how well predictions match measurements
  2. Adaptive damping alphas (xyz, yaw) — effective velocity decay per frame
  3. Tracker state over time (LOST=0, DETECTING=1, TRACKING=2, TEMP_LOST=3)

Usage (inside the Isaac ROS container, after sourcing the workspace):
    python3 tracker_visualizer.py
    # or
    ros2 run armor_tracker tracker_visualizer.py  (if installed)
"""

import collections
import threading

import matplotlib
matplotlib.use('TkAgg')  # non-blocking backend; falls back gracefully on headless
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from auto_aim_interfaces.msg import TrackerInfo, Target

HISTORY = 300  # ~10 s at 30 Hz


class TrackerVisualizer(Node):
    STATE_LABELS = {0: 'LOST', 1: 'DETECTING', 2: 'TRACKING', 3: 'TEMP_LOST'}
    STATE_COLORS = {0: 'red', 1: 'orange', 2: 'green', 3: 'yellow'}

    def __init__(self):
        super().__init__('tracker_visualizer')

        # Ring buffers
        self.t_info = collections.deque(maxlen=HISTORY)
        self.pos_diff = collections.deque(maxlen=HISTORY)
        self.yaw_diff = collections.deque(maxlen=HISTORY)
        self.xyz_alpha = collections.deque(maxlen=HISTORY)
        self.yaw_alpha = collections.deque(maxlen=HISTORY)

        self.t_target = collections.deque(maxlen=HISTORY)
        self.state = collections.deque(maxlen=HISTORY)

        self._t0 = None  # first timestamp (for relative time axis)
        self._lock = threading.Lock()

        # QoS: match the publishers
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        target_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        self.create_subscription(TrackerInfo, '/tracker/info', self._info_cb, info_qos)
        self.create_subscription(Target, '/tracker/target', self._target_cb, target_qos)

        self.get_logger().info('Tracker visualizer started — waiting for data...')

    # ── Callbacks ──────────────────────────────────────────────────────
    def _stamp_to_sec(self, stamp):
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _info_cb(self, msg: TrackerInfo):
        # TrackerInfo has no header — use node clock
        t = self._stamp_to_sec(self.get_clock().now().to_msg())
        with self._lock:
            self.t_info.append(t)
            self.pos_diff.append(msg.position_diff)
            self.yaw_diff.append(msg.yaw_diff)
            self.xyz_alpha.append(msg.xyz_damping_alpha)
            self.yaw_alpha.append(msg.yaw_damping_alpha)

    def _target_cb(self, msg: Target):
        t = self._stamp_to_sec(msg.header.stamp)
        with self._lock:
            self.t_target.append(t)
            self.state.append(msg.tracker_state)

    # ── Snapshot for the animation loop ────────────────────────────────
    def snapshot(self):
        with self._lock:
            return {
                't_info': list(self.t_info),
                'pos_diff': list(self.pos_diff),
                'yaw_diff': list(self.yaw_diff),
                'xyz_alpha': list(self.xyz_alpha),
                'yaw_alpha': list(self.yaw_alpha),
                't_target': list(self.t_target),
                'state': list(self.state),
            }


def main():
    rclpy.init()
    node = TrackerVisualizer()

    # Spin ROS in a background thread so matplotlib can own the main thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── Matplotlib setup ──────────────────────────────────────────────
    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
    fig.suptitle('Armor Tracker Debug', fontsize=13)
    fig.tight_layout(rect=[0.05, 0.03, 1, 0.95])

    ax_innov, ax_alpha, ax_state = axes

    # Subplot 1: Innovation
    line_pos, = ax_innov.plot([], [], 'b-', linewidth=1, label='pos_diff (m)')
    line_yaw, = ax_innov.plot([], [], 'r-', linewidth=1, label='yaw_diff (rad)')
    ax_innov.set_ylabel('Innovation')
    ax_innov.legend(loc='upper right', fontsize=8)
    ax_innov.set_ylim(0, 1.0)
    ax_innov.grid(True, alpha=0.3)

    # Subplot 2: Damping alphas
    line_xyz_a, = ax_alpha.plot([], [], 'g-', linewidth=1.5, label='xyz_alpha')
    line_yaw_a, = ax_alpha.plot([], [], 'm-', linewidth=1.5, label='yaw_alpha')
    ax_alpha.set_ylabel('Damping alpha')
    ax_alpha.legend(loc='upper right', fontsize=8)
    ax_alpha.set_ylim(0.5, 1.05)
    ax_alpha.grid(True, alpha=0.3)

    # Subplot 3: Tracker state
    scat_state = ax_state.scatter([], [], c=[], s=8, marker='s')
    ax_state.set_ylabel('State')
    ax_state.set_xlabel('Time (s)')
    ax_state.set_yticks([0, 1, 2, 3])
    ax_state.set_yticklabels(['LOST', 'DETECT', 'TRACK', 'T_LOST'], fontsize=7)
    ax_state.set_ylim(-0.5, 3.5)
    ax_state.grid(True, alpha=0.3)

    def update(_frame):
        d = node.snapshot()

        # Info data
        ti = d['t_info']
        if ti:
            line_pos.set_data(ti, d['pos_diff'])
            line_yaw.set_data(ti, d['yaw_diff'])
            line_xyz_a.set_data(ti, d['xyz_alpha'])
            line_yaw_a.set_data(ti, d['yaw_alpha'])

            # Auto-scale y for innovation
            max_innov = max(max(d['pos_diff'], default=0.1), max(d['yaw_diff'], default=0.1)) * 1.2
            ax_innov.set_ylim(0, max(max_innov, 0.15))

        # State data
        tt = d['t_target']
        if tt:
            colors = [TrackerVisualizer.STATE_COLORS.get(s, 'grey') for s in d['state']]
            offsets = list(zip(tt, d['state']))
            scat_state.set_offsets(offsets)
            scat_state.set_color(colors)

        # Shared x-axis: show last 10 seconds
        all_t = ti + tt
        if all_t:
            t_max = max(all_t)
            t_min = max(t_max - 10.0, 0)
            for ax in axes:
                ax.set_xlim(t_min, t_max + 0.2)

        return line_pos, line_yaw, line_xyz_a, line_yaw_a, scat_state

    _ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)  # noqa: F841
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
