"""
turret_idle_target_publisher.py

LAB VERSION.

Publishes /turret/idle_target for the CV turret_yaw_mux.

Lab default:
  target = centro_lab

YAML supported forms:

turret_idle_targets:
  red: centro_lab
  blue: centro_lab

or:

turret_idle_targets:
  red: {x: 3.508, y: 6.240}
  blue: {x: 3.508, y: 6.240}
"""

import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


class TurretIdleTargetPublisher(Node):
    def __init__(self):
        super().__init__("turret_idle_target_publisher")

        self.declare_parameter("team", "red")
        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("use_team_topic", True)
        self.declare_parameter("team_topic", "/team")
        self.declare_parameter("target_topic", "/turret/idle_target")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("default_target_name", "centro_lab")

        self.team = str(self.get_parameter("team").value).lower()
        self.wp_file = self.get_parameter("waypoints_file").value
        self.use_team_topic = bool(self.get_parameter("use_team_topic").value)
        self.team_topic = self.get_parameter("team_topic").value
        self.target_topic = self.get_parameter("target_topic").value
        self.default_target_name = self.get_parameter("default_target_name").value
        rate = float(self.get_parameter("publish_rate").value)

        if not self.wp_file:
            self.get_logger().error("waypoints_file is required")
            raise SystemExit(1)

        with open(self.wp_file, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.waypoints = self.cfg.get("waypoints", {})
        self.targets = self.cfg.get("turret_idle_targets", {})

        if self.default_target_name not in self.waypoints:
            self.get_logger().error(
                f'default_target_name "{self.default_target_name}" not found in {self.wp_file}'
            )
            raise SystemExit(1)

        if self.use_team_topic:
            self.create_subscription(String, self.team_topic, self._on_team, 10)

        self.pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)

        self.get_logger().info(
            f"turret_idle_target_publisher LAB started: team={self.team}, "
            f"default_target={self.default_target_name}, topic={self.target_topic}"
        )

    def _on_team(self, msg: String):
        t = msg.data.strip().lower()
        if t in ("red", "blue") and t != self.team:
            self.get_logger().info(f"team: {self.team} -> {t}")
            self.team = t

    def _resolve_target(self):
        spec = self.targets.get(self.team, self.default_target_name)

        if isinstance(spec, str):
            wp = self.waypoints.get(spec)

            if wp is None:
                self.get_logger().warn_once(
                    f'turret_idle_targets[{self.team}] references unknown waypoint "{spec}". '
                    f'Falling back to "{self.default_target_name}".'
                )
                wp = self.waypoints.get(self.default_target_name)

            if wp is None:
                return None

            return float(wp["x"]), float(wp["y"])

        if isinstance(spec, dict):
            return float(spec["x"]), float(spec["y"])

        wp = self.waypoints.get(self.default_target_name)
        if wp is None:
            return None

        return float(wp["x"]), float(wp["y"])

    def _tick(self):
        xy = self._resolve_target()
        if xy is None:
            return

        x, y = xy

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.point.x = x
        msg.point.y = y
        msg.point.z = 0.0

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TurretIdleTargetPublisher()
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()