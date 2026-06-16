"""
waypoint_manager – Execute named strategies from arena_waypoints.yaml.

Subscribes:
  /strategy   (std_msgs/String)   — switch strategy by name at runtime
                                    (e.g. "rush_center", "retreat_to_spawn",
                                    "hold_center_diagonal", "wait_at_spawn")
  /team       (std_msgs/String)   — "red" | "blue"  (if absent, uses launch
                                    parameter)

Publishes:
  /waypoint_manager/status (std_msgs/String)

The combination of /team + /strategy fully determines which waypoint sequence
to execute: arena_waypoints.yaml → strategies[<strategy>][<team>].

Switching strategy (or team) cancels the current Nav2 task and starts the new
one. Receiving the same strategy twice in a row is a no-op.
"""
import math
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(x, y, yaw, frame_id='map') -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame_id
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.z = math.sin(float(yaw) / 2.0)
    p.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return p


class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')

        self.declare_parameter('team', 'red')
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('initial_strategy', 'wait_at_spawn')

        self.team = self.get_parameter('team').value
        wp_file = self.get_parameter('waypoints_file').value
        self.current_strategy = self.get_parameter('initial_strategy').value

        if not wp_file:
            self.get_logger().error('waypoints_file is required')
            raise SystemExit(1)

        with open(wp_file, 'r') as f:
            self.cfg = yaml.safe_load(f)

        self.waypoints = self.cfg.get('waypoints', {})
        self.strategies = self.cfg.get('strategies', {})

        # Runtime overrides
        self.create_subscription(String, '/strategy', self._on_strategy, 10)
        self.create_subscription(String, '/team', self._on_team, 10)
        self.status_pub = self.create_publisher(
            String, '/waypoint_manager/status', 10)

        self.navigator = BasicNavigator()
        self._started = False
        self._task_in_progress = False
        self._reset_requested = False
        self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'waypoint_manager started. team={self.team} '
            f'initial_strategy={self.current_strategy}')

    def _on_strategy(self, msg: String):
        name = msg.data.strip()
        if name not in self.strategies:
            self.get_logger().warn(f'Unknown strategy: {name}')
            return
        if name == self.current_strategy and self._task_in_progress:
            return  # already running this

        if name != self.current_strategy:
            self.get_logger().info(
                f'Switching strategy: {self.current_strategy} → {name}')
        self.current_strategy = name

        if self._task_in_progress:
            self.navigator.cancelTask()
            self._reset_requested = True
        else:
            # Idle — start immediately
            self._reset_requested = True

    def _on_team(self, msg: String):
        new = msg.data.strip().lower()
        if new not in ('red', 'blue'):
            return
        if new != self.team:
            self.get_logger().info(f'Team change: {self.team} → {new}')
            self.team = new
            if self._task_in_progress:
                self.navigator.cancelTask()
                self._reset_requested = True

    def _tick(self):
        status = String()
        status.data = (f'team={self.team} strategy={self.current_strategy} '
                       f'task_running={self._task_in_progress}')
        self.status_pub.publish(status)

        if not self._started:
            self._started = True
            self.get_logger().info('Waiting for Nav2 to become active...')
            self.navigator.waitUntilNav2Active(
                localizer='amcl', navigator='bt_navigator')
            self.get_logger().info('Nav2 is ACTIVE.')
            self._execute_strategy()
            return

        if self._task_in_progress:
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                self._task_in_progress = False

                if self._reset_requested:
                    self._reset_requested = False
                    self._execute_strategy()
                elif result == TaskResult.SUCCEEDED:
                    self.get_logger().info(
                        f'Strategy "{self.current_strategy}" COMPLETED')
                    # Patrol / hold strategies loop back
                    if self.current_strategy in (
                            'hold_center_diagonal', 'patrol'):
                        self._execute_strategy()
                elif result == TaskResult.FAILED:
                    self.get_logger().warn(
                        f'Strategy "{self.current_strategy}" FAILED — retrying in 3 s')
                    self.create_timer(3.0, self._retry_once)
                elif result == TaskResult.CANCELED:
                    self.get_logger().info('Task canceled')
        elif self._reset_requested:
            self._reset_requested = False
            self._execute_strategy()

    def _retry_once(self):
        if not self._task_in_progress:
            self._execute_strategy()

    def _execute_strategy(self):
        strategy = self.strategies.get(self.current_strategy, {})
        wp_names = strategy.get(self.team, [])
        if not wp_names:
            self.get_logger().warn(
                f'Strategy "{self.current_strategy}" has no waypoints '
                f'for team "{self.team}"')
            return

        poses = []
        for name in wp_names:
            if name not in self.waypoints:
                self.get_logger().error(f'Unknown waypoint: {name}')
                return
            wp = self.waypoints[name]
            pose = make_pose(wp['x'], wp['y'], wp['yaw'])
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            poses.append(pose)

        self.get_logger().info(
            f'[{self.team}] executing "{self.current_strategy}" '
            f'({len(poses)} waypoints): {wp_names}')

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


if __name__ == '__main__':
    main()
