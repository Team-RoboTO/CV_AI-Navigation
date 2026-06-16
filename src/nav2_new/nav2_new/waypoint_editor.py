"""
waypoint_editor – Capture named waypoints by clicking on the map in RViz.

Workflow:
  Terminal A:
      ros2 run nav2_new waypoint_editor

  Terminal B  (set a name for the NEXT click):
      ./scripts/set_waypoint_name.sh blue_tunnel_1
      # — or equivalently —
      ros2 topic pub --once /waypoint_editor/next_name std_msgs/String \\
          "data: 'blue_tunnel_1'"

  In RViz:
      Use "2D Goal Pose"  (position + yaw)
      Or "Publish Point"  (position only, yaw=0)
      The click is labelled with the name you set, then the name is cleared.

  Names that are not set default to auto-numbered (wp_01, wp_02, …).

Output:
  Appends to ~/waypoints_captured.yaml — copy entries into
  config/arena_waypoints.yaml when done.
"""
import os
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class WaypointEditor(Node):
    def __init__(self):
        super().__init__('waypoint_editor')

        self.declare_parameter(
            'output_file',
            os.path.join(os.path.expanduser('~'), 'waypoints_captured.yaml'))
        self.declare_parameter('prefix', 'wp_')

        self.output_file = self.get_parameter('output_file').value
        self.prefix = self.get_parameter('prefix').value

        # Reset / initialize output file
        with open(self.output_file, 'w') as f:
            f.write('# Waypoints captured from RViz clicks\n')
            f.write('# Copy these entries into config/arena_waypoints.yaml\n')
            f.write('waypoints:\n')

        self.auto_counter = 0
        self.next_name = None   # set by /waypoint_editor/next_name

        # Inputs from RViz
        self.create_subscription(
            PointStamped, '/clicked_point', self._on_point, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._on_pose, 10)

        # Name input from user
        self.create_subscription(
            String, '/waypoint_editor/next_name', self._on_next_name, 10)

        # Markers for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray, '/waypoint_editor/markers', 10)
        self.markers = MarkerArray()

        self.get_logger().info('═══════════════════════════════════════════════════════════════')
        self.get_logger().info('  Waypoint Editor ready.')
        self.get_logger().info('  ▸ Set a name for the next click:')
        self.get_logger().info('      ros2 topic pub --once /waypoint_editor/next_name \\')
        self.get_logger().info('        std_msgs/String "data: \'my_waypoint\'"')
        self.get_logger().info('    (or use scripts/set_waypoint_name.sh <name>)')
        self.get_logger().info('  ▸ Click "2D Goal Pose" in RViz to capture position+yaw')
        self.get_logger().info('  ▸ Click "Publish Point" for position only')
        self.get_logger().info(f'  ▸ Output: {self.output_file}')
        self.get_logger().info('═══════════════════════════════════════════════════════════════')

    # ─── Callbacks ───────────────────────────────────────────────────────────

    def _on_next_name(self, msg: String):
        name = msg.data.strip()
        if not name:
            self.next_name = None
            self.get_logger().info('Next name CLEARED (will auto-number)')
        else:
            # Sanitize: YAML keys can't have special chars
            safe = ''.join(c if (c.isalnum() or c in '_-') else '_' for c in name)
            self.next_name = safe
            self.get_logger().info(
                f'Next click will be named: "{self.next_name}"')

    def _on_point(self, msg: PointStamped):
        self._record(msg.point.x, msg.point.y, 0.0)

    def _on_pose(self, msg: PoseStamped):
        yaw = quat_to_yaw(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        self._record(msg.pose.position.x, msg.pose.position.y, yaw)

    # ─── Recording ───────────────────────────────────────────────────────────

    def _record(self, x, y, yaw):
        if self.next_name:
            name = self.next_name
            self.next_name = None   # consume the name
        else:
            self.auto_counter += 1
            name = f'{self.prefix}{self.auto_counter:02d}'

        line = f'  {name}: {{ x: {x:.3f}, y: {y:.3f}, yaw: {yaw:.3f} }}\n'
        with open(self.output_file, 'a') as f:
            f.write(line)

        self.get_logger().info(
            f'✓ Captured "{name}" → x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}')
        self._add_marker(name, x, y, yaw)

    def _add_marker(self, name, x, y, yaw):
        marker_id_base = len(self.markers.markers)

        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.ns = 'waypoints'
        arrow.id = marker_id_base
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = x
        arrow.pose.position.y = y
        arrow.pose.position.z = 0.1
        arrow.pose.orientation.z = math.sin(yaw / 2.0)
        arrow.pose.orientation.w = math.cos(yaw / 2.0)
        arrow.scale.x = 0.4
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.a = 0.9
        arrow.color.r = 0.1
        arrow.color.g = 0.9
        arrow.color.b = 0.2
        self.markers.markers.append(arrow)

        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.ns = 'waypoint_labels'
        text.id = marker_id_base + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.35
        text.scale.z = 0.25
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = name
        self.markers.markers.append(text)

        self.marker_pub.publish(self.markers)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = WaypointEditor()
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
