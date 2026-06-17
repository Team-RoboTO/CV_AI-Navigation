"""
nav_match_reset.py

Lightweight reset helper for end-of-match / next-match preparation.

Listens:
  /nav_restart_requested std_msgs/Bool
  /match_ended std_msgs/Bool

Actions:
  - clears local/global costmaps when services are available
  - publishes /nav_match_reset_done

It intentionally does not kill/relaunch Nav2 nodes. ROS 2 launch cannot reliably
restart included launch descriptions from a normal node. The robust pattern is:
  sensors and Nav2 stay alive, FSM resets, costmaps are cleared, initial pose is
  republished by set_initial_pose.py.
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from nav2_msgs.srv import ClearEntireCostmap


class NavMatchReset(Node):
    def __init__(self):
        super().__init__('nav_match_reset')

        self.declare_parameter('clear_local_service', '/local_costmap/clear_entirely_local_costmap')
        self.declare_parameter('clear_global_service', '/global_costmap/clear_entirely_global_costmap')
        self.declare_parameter('min_reset_interval_sec', 3.0)
        self.declare_parameter('clear_on_match_ended', True)

        self.local_srv_name = self.get_parameter('clear_local_service').value
        self.global_srv_name = self.get_parameter('clear_global_service').value
        self.min_interval = float(self.get_parameter('min_reset_interval_sec').value)
        self.clear_on_match_ended = bool(self.get_parameter('clear_on_match_ended').value)

        self.local_cli = self.create_client(ClearEntireCostmap, self.local_srv_name)
        self.global_cli = self.create_client(ClearEntireCostmap, self.global_srv_name)
        self.pub_done = self.create_publisher(Bool, '/nav_match_reset_done', 10)
        self.pub_status = self.create_publisher(String, '/nav_match_reset/status', 10)

        self.last_reset = -1e9
        self.create_subscription(Bool, '/nav_restart_requested', self._on_reset_request, 10)
        if self.clear_on_match_ended:
            self.create_subscription(Bool, '/match_ended', self._on_match_ended, 10)

        self.get_logger().info('nav_match_reset ready')

    def _status(self, text):
        m = String(); m.data = text
        self.pub_status.publish(m)
        self.get_logger().info(text)

    def _on_match_ended(self, msg: Bool):
        if msg.data:
            self._do_reset('match_ended')

    def _on_reset_request(self, msg: Bool):
        if msg.data:
            self._do_reset('nav_restart_requested')

    def _try_clear(self, client, name):
        if not client.wait_for_service(timeout_sec=0.5):
            self._status(f'service not available: {name}')
            return False
        fut = client.call_async(ClearEntireCostmap.Request())
        self._status(f'clear requested: {name}')
        return True

    def _do_reset(self, reason):
        now = time.monotonic()
        if now - self.last_reset < self.min_interval:
            return
        self.last_reset = now
        self._status(f'nav reset start reason={reason}')
        ok_local = self._try_clear(self.local_cli, self.local_srv_name)
        ok_global = self._try_clear(self.global_cli, self.global_srv_name)
        done = Bool(); done.data = bool(ok_local or ok_global)
        self.pub_done.publish(done)
        self._status(f'nav reset done reason={reason} clear_any={done.data}')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = NavMatchReset()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
