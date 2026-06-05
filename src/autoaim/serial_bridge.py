#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, Int32
import serial
import struct
import threading
import time
from rclpy.qos import qos_profile_system_default


'''
ls /dev/tty{ACM,USB}*

sudo -S  chmod 666 /dev/ttyACM0
'''



# =============================================================================
# PACKET DEFINITIONS
# =============================================================================
#
# TX (Jetson -> Micro): 7 floats = 28 bytes
#   [0] Timestamp    (seconds since this node started)
#   [1] Yaw          absolute target yaw from AI [rad]
#   [2] Pitch        absolute target pitch from AI [rad]
#   [3] Shoot        shoot flag from AI
#   [4] NavX         from NAV, cmd_vel linear.x
#   [5] NavY         from NAV, cmd_vel linear.y
#   [6] NavAngle     from NAV, cmd_vel angular.z
#
# RX (Micro -> Jetson): 10 floats = 40 bytes
#   [0] Micro yaw    current measured yaw from micro [rad]
#   [1] Micro pitch  current measured pitch from micro [rad]
#   [2..9] Other micro status values
#
# This node republishes:
#   /micro_status    Float32MultiArray with RX first, then TX echo:
#                    RX [0] yaw, [1] pitch, [2] vx, [3] vy, ...
#                    TX echo appended after RX: ai_yaw, ai_pitch, shoot, nav_x, nav_y, nav_angle
#
# AUTO-RECONNECT:
#   If the micro is reflashed, the USB cable unplugged, or the port disappears,
#   this node automatically closes the dead port and retries every
#   serial_reconnect_interval seconds until it comes back. No need to restart
#   the node from the terminal.
# =============================================================================

TX_NUM_VALUES = 7
TX_PACKET_SIZE = TX_NUM_VALUES * 4          # 28 bytes
TX_STRUCT_FORMAT = '<' + ('f' * TX_NUM_VALUES)

RX_NUM_VALUES = 10
RX_PACKET_SIZE = RX_NUM_VALUES * 4          # 40 bytes
RX_STRUCT_FORMAT = '<' + ('f' * RX_NUM_VALUES)


# Able or disable the 2 pipelines informations
AI_PIPELINE  = True
NAV_PIPELINE = False



class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('micro_communications_node')

        # ------------------------------------------------------------------
        # Internal state: latest values from each topic.
        # Protected by a lock because the timer callback runs in a
        # separate thread from the subscription callbacks.
        # ------------------------------------------------------------------
        self._lock = threading.Lock()

        # Values from cmd_vel_AI
        self._ai_yaw   = 0.0
        self._ai_pitch = 0.0
        self._ai_shoot = 0.0   # angular.x: 1.0 = shoot

        # Values from cmd_vel_NAV
        self._nav_x     = 0.0  # linear.x
        self._nav_y     = 0.0  # linear.y
        self._nav_angle = 0.0  # angular.z


        self.create_subscription(
            Twist,
            'cmd_vel_AI',
            self._ai_callback,
            qos_profile_system_default
        )

        self.create_subscription(
            Twist,
            'cmd_vel_NAV',
            self._nav_callback,
            qos_profile_system_default
        )

        self._micro_status_pub = self.create_publisher(
            Float32MultiArray,
            '/micro_status',
            10
        )

        # ------------------------------------------------------------------
        # Serial port configuration
        # ------------------------------------------------------------------
        # Checking the correct port, run: ls /dev/tty{ACM,USB}*
        # For a stable name use a udev rule:

        # udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct"

        # SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY",
        #   SYMLINK+="robot_micro"

        self.t_start = time.monotonic()

        self._port = self.declare_parameter("serial_port", "/dev/ttyACM0").value
        self._baudrate = int(self.declare_parameter("serial_baudrate", 500000).value)
        # How often to retry when the serial port is down [seconds].
        self._reconnect_interval = float(
            self.declare_parameter("serial_reconnect_interval", 2.0).value)
        # After this many seconds of no valid RX packets, consider the link dead
        # and trigger a reconnect.
        self._rx_timeout = float(
            self.declare_parameter("serial_rx_timeout", 3.0).value)

        self._ser = None
        self._last_rx_time = time.monotonic()
        self._last_reconnect_attempt = 0.0
        self._consecutive_errors = 0

        self._open_serial()

        # ------------------------------------------------------------------
        # Timer: send TX packet + drain RX at 100 Hz (every 10 ms).
        # Adjust frequency as needed — printing to console slows things down.
        # ------------------------------------------------------------------
        tx_hz = float(self.declare_parameter("serial_tx_hz", 100.0).value)
        self._timer = self.create_timer(1.0 / max(tx_hz, 1.0), self._serial_callback)

    # ------------------------------------------------------------------
    # Serial open / reconnect
    # ------------------------------------------------------------------

    def _open_serial(self) -> bool:
        """Try to open the serial port. Returns True on success."""
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None

        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,  # Change to PARITY_NONE if micro doesn't use it
                stopbits=serial.STOPBITS_ONE,
                timeout=0.001
            )
            # Flush any stale data from a previous session
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._last_rx_time = time.monotonic()
            self._consecutive_errors = 0
            self.get_logger().info(
                f"Serial port {self._port} opened successfully at {self._baudrate} baud.")
            return True
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(
                f"Failed to open serial port {self._port}: {e} — will retry "
                f"every {self._reconnect_interval:.1f}s")
            self._ser = None
            return False

    def _close_and_schedule_reconnect(self, reason: str):
        """Close the port and let the next timer tick attempt reconnect."""
        self.get_logger().warn(f"Serial link lost ({reason}) — will reconnect.")
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._consecutive_errors = 0

    # ------------------------------------------------------------------
    # Subscription callbacks  —  just store latest values
    # ------------------------------------------------------------------

    def _ai_callback(self, msg: Twist):
        """Receive detection/turret commands from the AI pipeline."""
        if not AI_PIPELINE:
                return
        with self._lock:
            self._ai_shoot = msg.angular.x   # shoot flag
            self._ai_pitch = msg.angular.y   # absolute pitch target [rad]
            self._ai_yaw   = msg.angular.z   # absolute yaw target [rad]

    def _nav_callback(self, msg: Twist):
        """Receive navigation velocity commands."""
        if not NAV_PIPELINE:
                return
        with self._lock:
            self._nav_x     = msg.linear.x
            self._nav_y     = msg.linear.y
            self._nav_angle = msg.angular.z

    # ------------------------------------------------------------------
    # Timer callback  —  TX + RX every tick
    # ------------------------------------------------------------------

    def _serial_callback(self):

        # ── If serial is not open, try to reconnect periodically ──
        if self._ser is None or not self._ser.is_open:
            now = time.monotonic()
            if now - self._last_reconnect_attempt >= self._reconnect_interval:
                self._last_reconnect_attempt = now
                self.get_logger().info(
                    f"Attempting serial reconnect on {self._port}...")
                self._open_serial()
            return

        # ── Check for RX timeout (micro reflashed, cable unplugged, etc.) ──
        now = time.monotonic()
        if now - self._last_rx_time > self._rx_timeout:
            self._close_and_schedule_reconnect(
                f"no valid RX for {self._rx_timeout:.1f}s")
            self._last_reconnect_attempt = now
            return

        # --- Build TX packet ---
        try:
            with self._lock:
                tx_timestamp = time.monotonic() - self.t_start
                data_to_send = [
                    self._ai_yaw,
                    self._ai_pitch,
                    self._ai_shoot,
                    self._nav_x,
                    self._nav_y,
                    self._nav_angle,
                ]

            tx_bytes = struct.pack(TX_STRUCT_FORMAT, tx_timestamp, *data_to_send)
            self._ser.write(tx_bytes)
            self._consecutive_errors = 0
        except (serial.SerialException, OSError) as e:
            self._consecutive_errors += 1
            self.get_logger().warn(
                f"Serial TX error ({self._consecutive_errors}): {e}")
            if self._consecutive_errors >= 5:
                self._close_and_schedule_reconnect(
                    f"{self._consecutive_errors} consecutive TX/RX errors")
                self._last_reconnect_attempt = now
            return

        # --- Drain RX buffer ---
        try:
            if self._ser.in_waiting >= RX_PACKET_SIZE:
                # Drain to the most recent complete packet.
                # If packets have stacked up (timer jitter, burst from micro), discard
                # all but the last one so the auto-aim node always gets fresh IMU data.
                available = self._ser.in_waiting
                stale_bytes = (available // RX_PACKET_SIZE - 1) * RX_PACKET_SIZE
                if stale_bytes > 0:
                    self._ser.read(stale_bytes)   # discard old packets
                rx_bytes = self._ser.read(RX_PACKET_SIZE)

                if len(rx_bytes) == RX_PACKET_SIZE:
                    try:
                        unpacked = struct.unpack(RX_STRUCT_FORMAT, rx_bytes)
                        # unpacked[0] = micro yaw [rad]
                        # unpacked[1] = micro pitch [rad]

                        # Basic sanity check: angles should be finite and reasonable.
                        # If the micro was reflashed mid-stream, garbage bytes produce
                        # huge float values; reject and flush to re-sync.
                        if all(abs(v) < 1e6 for v in unpacked[:2]):
                            # Single status topic used by autoaim and viewers.
                            # Expected indices: [0] yaw, [1] pitch, [2] vx, [3] vy.
                            status_msg = Float32MultiArray()
                            with self._lock:
                                tx_echo = [
                                    self._ai_yaw,
                                    self._ai_pitch,
                                    self._ai_shoot,
                                    self._nav_x,
                                    self._nav_y,
                                    self._nav_angle,
                                ]
                            status_msg.data = list(unpacked) + tx_echo
                            self._micro_status_pub.publish(status_msg)

                            self._last_rx_time = time.monotonic()
                            self._consecutive_errors = 0
                        else:
                            self.get_logger().warn(
                                "RX sanity check failed (garbage values) — flushing buffer")
                            self._ser.reset_input_buffer()

                    except struct.error as e:
                        self.get_logger().warn(f"RX unpack error: {e}")
                        # Flush the buffer on unpack error to try to re-align
                        self._ser.reset_input_buffer()

        except (serial.SerialException, OSError) as e:
            self._consecutive_errors += 1
            self.get_logger().warn(
                f"Serial RX error ({self._consecutive_errors}): {e}")
            if self._consecutive_errors >= 5:
                self._close_and_schedule_reconnect(
                    f"{self._consecutive_errors} consecutive TX/RX errors")
                self._last_reconnect_attempt = now


    def destroy_node(self):
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
