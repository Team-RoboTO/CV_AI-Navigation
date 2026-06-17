
"""
waypoint_manager.py

LAB VERSION.

Executes named strategies from arena_waypoints_lab.yaml.

Subscribes:
  /strategy std_msgs/String   strategy name to execute
  /team std_msgs/String       red | blue

Publishes:
  /waypoint_manager/status             std_msgs/String
  /waypoint_manager/strategy_started   std_msgs/String
  /waypoint_manager/strategy_completed std_msgs/String
  /waypoint_manager/strategy_failed    std_msgs/String
"""

import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


def make_pose(x, y, yaw, frame_id="map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        self.declare_parameter("team", "red")
        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("initial_strategy", "wait_at_spawn")
        self.declare_parameter(
            "loop_strategies",
            [
                "center_free_strategy",
                "hold_center_diagonal",
                "enemy_center_strategy",
                "percorso",
            ],
        )
        self.declare_parameter("retry_failed_after_sec", 3.0)
        self.declare_parameter("autostart_initial_strategy", True)

        self.team = str(self.get_parameter("team").value).lower()
        self.wp_file = self.get_parameter("waypoints_file").value
        self.current_strategy = self.get_parameter("initial_strategy").value
        self.loop_strategies = set(self.get_parameter("loop_strategies").value)
        self.retry_failed_after_sec = float(self.get_parameter("retry_failed_after_sec").value)
        self.autostart_initial_strategy = bool(self.get_parameter("autostart_initial_strategy").value)

        if not self.wp_file:
            self.get_logger().error("waypoints_file is required")
            raise SystemExit(1)

        with open(self.wp_file, "r") as f:
            self.cfg = yaml.safe_load(f)

        self.waypoints = self.cfg.get("waypoints", {})
        self.strategies = self.cfg.get("strategies", {})

        if self.current_strategy not in self.strategies:
            self.get_logger().error(
                f'initial_strategy "{self.current_strategy}" not found in {self.wp_file}'
            )
            raise SystemExit(1)

        missing_loop = [s for s in self.loop_strategies if s not in self.strategies]
        if missing_loop:
            self.get_logger().warn(
                f"loop_strategies not found in YAML and will be ignored: {missing_loop}"
            )
            self.loop_strategies = {s for s in self.loop_strategies if s in self.strategies}

        self.create_subscription(String, "/strategy", self._on_strategy, 10)
        self.create_subscription(String, "/team", self._on_team, 10)

        self.status_pub = self.create_publisher(String, "/waypoint_manager/status", 10)
        self.started_pub = self.create_publisher(String, "/waypoint_manager/strategy_started", 10)
        self.completed_pub = self.create_publisher(String, "/waypoint_manager/strategy_completed", 10)
        self.failed_pub = self.create_publisher(String, "/waypoint_manager/strategy_failed", 10)

        self.navigator = BasicNavigator()
        self._nav2_ready = False
        self._task_in_progress = False
        self._reset_requested = False
        self._retry_timer = None
        self._last_started_strategy = None

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"waypoint_manager LAB started. team={self.team}, "
            f"waypoints_file={self.wp_file}, initial_strategy={self.current_strategy}"
        )

    def _publish_string(self, pub, text):
        m = String()
        m.data = text
        pub.publish(m)

    def _on_strategy(self, msg: String):
        name = msg.data.strip()
        if name not in self.strategies:
            self.get_logger().warn(f"Unknown strategy: {name}")
            return

        if name == self.current_strategy and self._task_in_progress:
            return

        self.get_logger().info(f"Strategy request: {self.current_strategy} -> {name}")
        self.current_strategy = name

        if self._task_in_progress:
            self.navigator.cancelTask()
            self._reset_requested = True
        else:
            self._reset_requested = True

    def _on_team(self, msg: String):
        new = msg.data.strip().lower()
        if new not in ("red", "blue"):
            return

        if new != self.team:
            self.get_logger().info(f"Team change: {self.team} -> {new}")
            self.team = new
            if self._task_in_progress:
                self.navigator.cancelTask()
            self._reset_requested = True

    def _tick(self):
        status = String()
        status.data = (
            f"team={self.team} strategy={self.current_strategy} "
            f"task_running={self._task_in_progress} nav2_ready={self._nav2_ready}"
        )
        self.status_pub.publish(status)

        if not self._nav2_ready:
            self._nav2_ready = True
            self.get_logger().info("Waiting for Nav2 to become active...")
            self.navigator.waitUntilNav2Active(localizer="amcl", navigator="bt_navigator")
            self.get_logger().info("Nav2 is ACTIVE.")

            if self.autostart_initial_strategy:
                self._execute_strategy()
            return

        if self._task_in_progress:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                finished_strategy = self._last_started_strategy or self.current_strategy
                self._task_in_progress = False

                if self._reset_requested:
                    self._reset_requested = False
                    self._execute_strategy()
                    return

                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'Strategy "{finished_strategy}" COMPLETED')
                    self._publish_string(self.completed_pub, finished_strategy)

                    if finished_strategy in self.loop_strategies and finished_strategy == self.current_strategy:
                        self._execute_strategy()

                elif result == TaskResult.FAILED:
                    self.get_logger().warn(
                        f'Strategy "{finished_strategy}" FAILED — '
                        f"retrying in {self.retry_failed_after_sec:.1f}s"
                    )
                    self._publish_string(self.failed_pub, finished_strategy)
                    self._retry_timer = self.create_timer(
                        self.retry_failed_after_sec,
                        self._retry_once,
                    )

                elif result == TaskResult.CANCELED:
                    self.get_logger().info(f'Strategy "{finished_strategy}" canceled')

        elif self._reset_requested:
            self._reset_requested = False
            self._execute_strategy()

    def _retry_once(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

        if not self._task_in_progress:
            self._execute_strategy()

    def _execute_strategy(self):
        strategy = self.strategies.get(self.current_strategy, {})
        wp_names = strategy.get(self.team, [])

        if not wp_names:
            self.get_logger().warn(
                f'Strategy "{self.current_strategy}" has no waypoints for team "{self.team}"'
            )
            return

        poses = []
        for name in wp_names:
            if name not in self.waypoints:
                self.get_logger().error(
                    f'Unknown waypoint "{name}" in strategy "{self.current_strategy}"'
                )
                return

            wp = self.waypoints[name]
            pose = make_pose(wp["x"], wp["y"], wp.get("yaw", 0.0))
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            poses.append(pose)

        self.get_logger().info(
            f'[{self.team}] executing "{self.current_strategy}" '
            f"({len(poses)} waypoint/s): {wp_names}"
        )

        self._publish_string(self.started_pub, self.current_strategy)
        self._last_started_strategy = self.current_strategy

        if len(poses) == 1:
            self.navigator.goToPose(poses[0])
        else:
            self.navigator.goThroughPoses(poses)

        self._task_in_progress = True


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WaypointManager()
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