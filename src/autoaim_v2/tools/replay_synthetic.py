#!/usr/bin/env python3
"""End-to-end bench check for aim_node WITHOUT camera/serial hardware.

Publishes a synthetic spinning+strafing car as /detector/armors_keypoints
(projected through the same camera model the solver uses) plus /camera_info,
then verifies on /aimv2/debug that the node reaches TRACKING and estimates
spin rate, distance and regime correctly.

Run (two terminals, or let this script spawn nothing):
  ros2 run autoaim_v2 aim_node --ros-args -p input_mode:=ros_topics \
      -p sim_gimbal:=true -p fixed_target_class:=0 -p debug_enable:=true \
      -p debug_every:=1
  python3 tools/replay_synthetic.py [--omega 25] [--duration 4]

Exit code 0 = all checks passed.
"""
import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray
from autoaim.msg import ArmorKeypoint, ArmorKeypointArray

FX = FY = 730.0
CX, CY = 480.0, 300.0
W, H = 960, 600
CAM_H = 0.42  # must match gimbal_height in the node params
ARMOR_W, ARMOR_H = 0.135, 0.056
ARMOR_PITCH = math.radians(15)


def plate_corners_world(pos, theta_a):
    cy, sy = math.cos(theta_a), math.sin(theta_a)
    cr, sr = math.cos(ARMOR_PITCH), math.sin(ARMOR_PITCH)
    R = np.array([[cy * cr, -sy, cy * sr],
                  [sy * cr, cy, sy * sr],
                  [-sr, 0, cr]])
    hy, hz = ARMOR_W / 2, ARMOR_H / 2
    obj = np.array([[0, hy, hz], [0, -hy, hz], [0, -hy, -hz], [0, hy, -hz]])
    return pos[None, :] + obj @ R.T


def project(p_world):
    """world -> pixel for zero gimbal angles, camera at (0,0,CAM_H)."""
    rel = p_world - np.array([0.0, 0.0, CAM_H])
    p_cam = np.array([-rel[1], -rel[2], rel[0]])  # world->optical, zero angles
    if p_cam[2] <= 0.05:
        return None
    return (FX * p_cam[0] / p_cam[2] + CX, FY * p_cam[1] / p_cam[2] + CY)


class Replay(Node):
    def __init__(self, args):
        super().__init__("aimv2_replay")
        self.args = args
        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        latched = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.kpt_pub = self.create_publisher(ArmorKeypointArray,
                                             "/detector/armors_keypoints", qos)
        self.cam_pub = self.create_publisher(CameraInfo, "/camera_info", latched)
        self.dbg_sub = self.create_subscription(Float32MultiArray, "/aimv2/debug",
                                                self.on_debug, qos)
        info = CameraInfo()
        info.width, info.height = W, H
        info.k = [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0]
        info.header.frame_id = "camera_color_optical_frame"
        self.cam_pub.publish(info)

        # Simulated car.
        self.c = np.array([3.0, 0.0])
        self.v = np.array([args.vx, args.vy])
        self.z = 0.25
        self.theta = 0.2
        self.omega = args.omega
        self.r = 0.20

        self.t_prev = time.monotonic()
        self.debug_rows = []
        self.timer = self.create_timer(1.0 / 120, self.tick)

    def on_debug(self, msg):
        self.debug_rows.append(list(msg.data))

    def tick(self):
        now = time.monotonic()
        dt = now - self.t_prev
        self.t_prev = now
        self.c += self.v * dt
        # Bounce inside the camera FOV so the target stays observable, like a
        # real 1v1 opponent juking in front of the robot.
        if abs(self.c[1]) > 1.2:
            self.c[1] = math.copysign(1.2, self.c[1])
            self.v[1] = -self.v[1]
        self.theta = math.atan2(math.sin(self.theta + self.omega * dt),
                                math.cos(self.theta + self.omega * dt))

        arr = ArmorKeypointArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "camera_color_optical_frame"
        arr.image_width, arr.image_height = W, H
        arr.keypoint_order = ["TL", "TR", "BR", "BL"]

        for i in range(4):
            phi = self.theta + i * math.pi / 2
            pos = np.array([self.c[0] - self.r * math.cos(phi),
                            self.c[1] - self.r * math.sin(phi), self.z])
            bearing = math.atan2(pos[1], pos[0])
            obl = abs(math.atan2(math.sin(phi - bearing), math.cos(phi - bearing)))
            if obl > math.radians(60):
                continue
            px = [project(p) for p in plate_corners_world(pos, phi)]
            # A real camera only sees corners inside the sensor.
            if any(p is None or not (0 <= p[0] < W and 0 <= p[1] < H)
                   for p in px):
                continue
            k = ArmorKeypoint()
            k.header = arr.header
            k.class_id = 0  # blue
            k.class_name = "blue_armor"
            k.confidence = 0.9
            k.keypoints_xy = [float(v) for p in px for v in p]
            k.keypoint_scores = [0.9] * 4
            k.keypoint_valid = [True] * 4
            xs = [p[0] for p in px]
            ys = [p[1] for p in px]
            k.bbox_cx = float(sum(xs) / 4)
            k.bbox_cy = float(sum(ys) / 4)
            k.bbox_w = float(max(xs) - min(xs))
            k.bbox_h = float(max(ys) - min(ys))
            arr.detections.append(k)

        self.kpt_pub.publish(arr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--omega", type=float, default=25.0, help="spin [rad/s]")
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=4.0)
    args = ap.parse_args()

    rclpy.init()
    node = Replay(args)
    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.02)

    rows = node.debug_rows
    node.destroy_node()
    rclpy.shutdown()

    if len(rows) < 20:
        print(f"FAIL: only {len(rows)} debug rows — is aim_node running with "
              "debug_enable:=true / debug_every:=1?")
        return 1

    tail = rows[len(rows) // 2:]
    state = [r[1] for r in tail]
    omega = [r[4] for r in tail]
    dist = [r[5] for r in tail]
    regime = [r[0] for r in tail]

    ok = True
    tracking_frac = sum(1 for s in state if s == 2) / len(state)
    print(f"rows={len(rows)}  TRACKING fraction (2nd half): {tracking_frac:.2f}")
    if tracking_frac < 0.9:
        print("FAIL: tracker did not stay in TRACKING")
        ok = False

    med_omega = sorted(omega)[len(omega) // 2]
    print(f"estimated omega: {med_omega:+.2f} rad/s  (true {args.omega:+.2f})")
    if abs(med_omega - args.omega) > max(0.15 * abs(args.omega), 1.0):
        print("FAIL: omega estimate off")
        ok = False

    med_dist = sorted(dist)[len(dist) // 2]
    print(f"aim distance: {med_dist:.2f} m (car center ~3 m)")
    if not (2.3 < med_dist < 3.8):
        print("FAIL: distance estimate off")
        ok = False

    want = 3 if abs(args.omega) >= 12 else (1 if abs(args.omega) < 2 else 2)
    med_regime = sorted(regime)[len(regime) // 2]
    print(f"fire regime: {med_regime:.0f} (expected {want})")
    if med_regime != want:
        print("FAIL: wrong fire regime")
        ok = False

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
