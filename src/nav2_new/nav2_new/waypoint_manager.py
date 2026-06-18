#!/usr/bin/env python3
"""
waypoint_manager.py

LAB / SEQUENTIAL VERSION.

Esegue strategie definite in arena_waypoints_lab.yaml.

Cambio importante:
  Se una strategia ha più waypoint, questo nodo li manda a Nav2 UNO ALLA VOLTA
  usando goToPose(). Non usa goThroughPoses() di default.

Perché:
  goThroughPoses() crea un unico task multi-pose. In mappe strette può causare
  oscillazioni vicino ai waypoint intermedi. Con goToPose() sequenziale:
    waypoint 1 -> successo -> waypoint 2 -> successo -> ...

Subscribes:
  /strategy std_msgs/String   nome strategia
  /team     std_msgs/String   red | blue

Publishes:
  /waypoint_manager/status
  /waypoint_manager/strategy_started
  /waypoint_manager/strategy_completed
  /waypoint_manager/strategy_failed
"""

import math
import time
import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


def make_pose(x, y, yaw, frame_id="map") -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)

    return pose


class WaypointManager(Node):
    def __init__(self):
        super().__init__("waypoint_manager")

        self.declare_parameter("team", "red")
        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("initial_strategy", "wait_at_spawn")
        self.declare_parameter("retry_failed_after_sec", 3.0)
        self.declare_parameter("autostart_initial_strategy", True)

        # true  -> manda i waypoint uno alla volta con goToPose()
        # false -> vecchio comportamento: goThroughPoses() per strategie multi-waypoint
        self.declare_parameter("execute_waypoints_sequentially", True)

        self.declare_parameter(
            "loop_strategies",
            [
                "enemy_center_strategy",
                "percorso",
            ],
        )

        self.team = str(self.get_parameter("team").value).lower()
        self.wp_file = str(self.get_parameter("waypoints_file").value)
        self.current_strategy = str(self.get_parameter("initial_strategy").value)
        self.retry_failed_after_sec = float(
            self.get_parameter("retry_failed_after_sec").value
        )
        self.autostart_initial_strategy = bool(
            self.get_parameter("autostart_initial_strategy").value
        )
        self.execute_waypoints_sequentially = bool(
            self.get_parameter("execute_waypoints_sequentially").value
        )

        self.loop_strategies = set(self.get_parameter("loop_strategies").value)

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
            self.loop_strategies = {
                s for s in self.loop_strategies if s in self.strategies
            }

        self.create_subscription(String, "/strategy", self._on_strategy, 10)
        self.create_subscription(String, "/team", self._on_team, 10)

        self.status_pub = self.create_publisher(
            String,
            "/waypoint_manager/status",
            10,
        )
        self.started_pub = self.create_publisher(
            String,
            "/waypoint_manager/strategy_started",
            10,
        )
        self.completed_pub = self.create_publisher(
            String,
            "/waypoint_manager/strategy_completed",
            10,
        )
        self.failed_pub = self.create_publisher(
            String,
            "/waypoint_manager/strategy_failed",
            10,
        )

        self.navigator = BasicNavigator()

        # IMPORTANTE:
        # Non usiamo:
        #   self.navigator.waitUntilNav2Active(localizer="amcl")
        # né:
        #   self.navigator.waitUntilNav2Active(localizer="robot_localization")
        #
        # Perché:
        # - con amcl il BasicNavigator può provare a pubblicare initial pose a 0,0,0;
        # - con robot_localization aspetta /robot_localization/get_state, che qui non esiste.
        #
        # Aspettiamo solo che bt_navigator sia ACTIVE.
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.bt_state_future = None
        self.last_nav2_wait_log = 0.0

        self._nav2_ready = False
        self._task_in_progress = False
        self._reset_requested = False
        self._retry_timer = None
        self._last_started_strategy = None

        # Stato esecuzione sequenziale.
        self._pose_queue = []
        self._waypoint_name_queue = []
        self._current_wp_index = 0

        self.create_timer(0.5, self._tick)

        self.get_logger().info(
            f"waypoint_manager started. team={self.team}, "
            f"waypoints_file={self.wp_file}, "
            f"initial_strategy={self.current_strategy}, "
            f"sequential={self.execute_waypoints_sequentially}"
        )

    def _publish_string(self, pub, text: str):
        msg = String()
        msg.data = text
        pub.publish(msg)

    def _log_wait_throttled(self, text: str, period_sec: float = 2.0):
        now = time.monotonic()
        if now - self.last_nav2_wait_log >= period_sec:
            self.get_logger().info(text)
            self.last_nav2_wait_log = now

    def _check_bt_navigator_active(self) -> bool:
        if not self.bt_state_client.service_is_ready():
            self._log_wait_throttled(
                "/bt_navigator/get_state service not available, waiting..."
            )
            return False

        if self.bt_state_future is None:
            req = GetState.Request()
            self.bt_state_future = self.bt_state_client.call_async(req)
            return False

        if not self.bt_state_future.done():
            return False

        try:
            result = self.bt_state_future.result()
        except Exception as exc:
            self.get_logger().warn(f"Failed to get bt_navigator state: {exc}")
            self.bt_state_future = None
            return False

        self.bt_state_future = None

        if result is None:
            self._log_wait_throttled("bt_navigator state result is None, waiting...")
            return False

        state = result.current_state

        if state.id == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info("bt_navigator is ACTIVE")
            return True

        self._log_wait_throttled(
            f"bt_navigator current state: {state.label}, waiting..."
        )
        return False

    def _clear_sequence(self):
        self._pose_queue = []
        self._waypoint_name_queue = []
        self._current_wp_index = 0

    def _on_strategy(self, msg: String):
        name = msg.data.strip()

        if not name:
            return

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
        new_team = msg.data.strip().lower()

        if new_team not in ("red", "blue"):
            return

        if new_team != self.team:
            self.get_logger().info(f"Team change: {self.team} -> {new_team}")
            self.team = new_team

            if self._task_in_progress:
                self.navigator.cancelTask()

            self._reset_requested = True

    def _tick(self):
        status = String()
        status.data = (
            f"team={self.team} "
            f"strategy={self.current_strategy} "
            f"task_running={self._task_in_progress} "
            f"nav2_ready={self._nav2_ready} "
            f"sequential={self.execute_waypoints_sequentially} "
            f"wp_index={self._current_wp_index}/{len(self._pose_queue)}"
        )
        self.status_pub.publish(status)

        if not self._nav2_ready:
            if self._check_bt_navigator_active():
                self._nav2_ready = True
                self.get_logger().info("Nav2 bt_navigator is ACTIVE.")

                if self.autostart_initial_strategy:
                    self._execute_strategy()

            return

        if self._task_in_progress:
            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            finished_strategy = self._last_started_strategy or self.current_strategy
            self._task_in_progress = False

            if self._reset_requested:
                self._reset_requested = False
                self._execute_strategy()
                return

            if result == TaskResult.SUCCEEDED:
                if self.execute_waypoints_sequentially and self._pose_queue:
                    wp_name = self._waypoint_name_queue[self._current_wp_index]
                    self.get_logger().info(
                        f"Waypoint completed: {wp_name} "
                        f"({self._current_wp_index + 1}/{len(self._pose_queue)})"
                    )

                    self._current_wp_index += 1

                    if self._current_wp_index < len(self._pose_queue):
                        self._start_next_pose_in_sequence()
                        return

                self.get_logger().info(f'Strategy "{finished_strategy}" COMPLETED')
                self._publish_string(self.completed_pub, finished_strategy)
                self._clear_sequence()

                if (
                    finished_strategy in self.loop_strategies
                    and finished_strategy == self.current_strategy
                ):
                    self._execute_strategy()

            elif result == TaskResult.FAILED:
                self.get_logger().warn(
                    f'Strategy "{finished_strategy}" FAILED — '
                    f"retrying in {self.retry_failed_after_sec:.1f}s"
                )
                self._publish_string(self.failed_pub, finished_strategy)
                self._clear_sequence()

                if self._retry_timer is not None:
                    self._retry_timer.cancel()
                    self._retry_timer = None

                self._retry_timer = self.create_timer(
                    self.retry_failed_after_sec,
                    self._retry_once,
                )

            elif result == TaskResult.CANCELED:
                self.get_logger().info(f'Strategy "{finished_strategy}" CANCELED')
                self._clear_sequence()

            else:
                self.get_logger().warn(
                    f'Strategy "{finished_strategy}" ended with unknown result: {result}'
                )
                self._publish_string(self.failed_pub, finished_strategy)
                self._clear_sequence()

        elif self._reset_requested:
            self._reset_requested = False
            self._execute_strategy()

    def _retry_once(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

        if not self._task_in_progress:
            self._execute_strategy()

    def _start_next_pose_in_sequence(self) -> bool:
        if self._current_wp_index >= len(self._pose_queue):
            return False

        pose = self._pose_queue[self._current_wp_index]
        wp_name = self._waypoint_name_queue[self._current_wp_index]

        pose.header.stamp = self.navigator.get_clock().now().to_msg()

        self.get_logger().info(
            f'[{self.team}] strategy="{self.current_strategy}" '
            f"going to waypoint "
            f"{self._current_wp_index + 1}/{len(self._pose_queue)}: {wp_name}"
        )

        self.navigator.goToPose(pose)
        self._task_in_progress = True
        return True

    def _execute_strategy(self):
        strategy = self.strategies.get(self.current_strategy, {})
        wp_names = strategy.get(self.team, [])

        if not wp_names:
            self.get_logger().warn(
                f'Strategy "{self.current_strategy}" has no waypoints '
                f'for team "{self.team}"'
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

            pose = make_pose(
                wp["x"],
                wp["y"],
                wp.get("yaw", 0.0),
                frame_id="map",
            )
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            poses.append(pose)

        self.get_logger().info(
            f'[{self.team}] executing "{self.current_strategy}" '
            f"({len(poses)} waypoint/s): {wp_names}"
        )

        self._publish_string(self.started_pub, self.current_strategy)
        self._last_started_strategy = self.current_strategy

        if self.execute_waypoints_sequentially:
            self._pose_queue = poses
            self._waypoint_name_queue = list(wp_names)
            self._current_wp_index = 0

            started = self._start_next_pose_in_sequence()

            if not started:
                self.get_logger().warn(
                    f'Cannot start strategy "{self.current_strategy}"'
                )
                self._clear_sequence()

        else:
            self._clear_sequence()

            if len(poses) == 1:
                self.navigator.goToPose(poses[0])
            else:
                self.navigator.goThroughPoses(poses)

            self._task_in_progress = True


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = WaypointManager()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except SystemExit:
        pass

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()