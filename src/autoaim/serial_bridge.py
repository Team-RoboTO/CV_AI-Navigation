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

# =============================================================================
# OPTIONAL FRAMED PROTOCOL (use_framed_protocol: True)
#
# The raw protocol has NO sync marker and NO checksum. If the node ever starts
# reading mid-packet (the micro streams continuously, so the very first read
# after a connect can land anywhere), the stream stays misaligned FOREVER,
# because reads are always exact multiples of 40 bytes. Misaligned floats often
# pass the old |v| < 1e6 sanity check, silently feeding garbage yaw/pitch into
# the whole aim pipeline.
#
# Framed format (REQUIRES matching micro firmware — see CHANGES.md):
#   TX:  0xA5 0x5A | 28-byte payload | CRC8(payload)   = 31 bytes
#   RX:  0x5A 0xA5 | 40-byte payload | CRC8(payload)   = 43 bytes
# The RX parser scans for the header and verifies the CRC, so it re-syncs
# automatically after any dropped byte.
# =============================================================================
TX_FRAME_HEADER = b'\xA5\x5A'
RX_FRAME_HEADER = b'\x5A\xA5'
RX_FRAME_SIZE = len(RX_FRAME_HEADER) + RX_PACKET_SIZE + 1   # 43 bytes
RX_BUFFER_CAP = RX_FRAME_SIZE * 64


def crc8(data: bytes) -> int:
    """CRC-8 (poly 0x07, init 0x00) — same as the snippet in CHANGES.md for the micro."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc


# Able or disable the 2 pipelines informations
AI_PIPELINE  = True
NAV_PIPELINE = False



class SerialBridgeNode(Node):

    def __init__(self):
        super().__init__('micro_communications_node')
        # Launch files keep this false in competition. The C++ serial bridge is
        # controlled by ROS arguments; this fallback Python bridge skips active
        # log calls and f-string construction with the same category flag.
        self._enable_ros_logs = bool(self.declare_parameter("enable_ros_logs", False).value)

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

        # SAFETY WATCHDOG: if /cmd_vel_AI goes stale (auto-aim node crashed,
        # detector died, ...), the shoot flag is forced to 0 after this many
        # seconds. Without this, the bridge re-transmits the LAST shoot flag at
        # 100 Hz forever — a robot that keeps firing with a dead brain.
        self._cmd_timeout = float(
            self.declare_parameter("cmd_timeout", 0.3).value)

        # Framed protocol with header + CRC8 (requires matching micro firmware).
        # Leave False until the micro side is updated — see CHANGES.md.
        self._use_framed = bool(
            self.declare_parameter("use_framed_protocol", False).value)

        self._last_ai_time = 0.0   # monotonic time of last /cmd_vel_AI message
        self._rx_buf = bytearray() # framed-mode receive buffer

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
            if self._enable_ros_logs:
                self.get_logger().info(
                    f"Serial port {self._port} opened successfully at {self._baudrate} baud.")
            return True
        except (serial.SerialException, OSError) as e:
            if self._enable_ros_logs:
                self.get_logger().warn(
                    f"Failed to open serial port {self._port}: {e} — will retry "
                    f"every {self._reconnect_interval:.1f}s")
            self._ser = None
            return False

    def _close_and_schedule_reconnect(self, reason: str):
        """Close the port and let the next timer tick attempt reconnect."""
        if self._enable_ros_logs:
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
            self._last_ai_time = time.monotonic()

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
                if self._enable_ros_logs:
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
                # SAFETY: force shoot=0 when /cmd_vel_AI is stale. Yaw/pitch are
                # still re-sent (the gimbal holds position) but firing stops.
                ai_stale = (time.monotonic() - self._last_ai_time) > self._cmd_timeout
                shoot = 0.0 if ai_stale else self._ai_shoot
                data_to_send = [
                    self._ai_yaw,
                    self._ai_pitch,
                    shoot,
                    self._nav_x,
                    self._nav_y,
                    self._nav_angle,
                ]

            payload = struct.pack(TX_STRUCT_FORMAT, tx_timestamp, *data_to_send)
            if self._use_framed:
                tx_bytes = TX_FRAME_HEADER + payload + bytes([crc8(payload)])
            else:
                tx_bytes = payload
            self._ser.write(tx_bytes)
            self._consecutive_errors = 0
        except (serial.SerialException, OSError) as e:
            self._consecutive_errors += 1
            if self._enable_ros_logs:
                self.get_logger().warn(
                    f"Serial TX error ({self._consecutive_errors}): {e}")
            if self._consecutive_errors >= 5:
                self._close_and_schedule_reconnect(
                    f"{self._consecutive_errors} consecutive TX/RX errors")
                self._last_reconnect_attempt = now
            return

        # --- Drain RX buffer ---
        try:
            if self._use_framed:
                self._drain_rx_framed()
            else:
                self._drain_rx_legacy()
        except (serial.SerialException, OSError) as e:
            self._consecutive_errors += 1
            if self._enable_ros_logs:
                self.get_logger().warn(
                    f"Serial RX error ({self._consecutive_errors}): {e}")
            if self._consecutive_errors >= 5:
                self._close_and_schedule_reconnect(
                    f"{self._consecutive_errors} consecutive TX/RX errors")
                self._last_reconnect_attempt = now

    # ------------------------------------------------------------------
    # RX parsing
    # ------------------------------------------------------------------

    def _sanity_ok(self, unpacked) -> bool:
        """Reject obviously corrupted packets (misalignment, reflash garbage).

        Misaligned float bytes frequently decode to plausible-looking small
        numbers, so the old |v| < 1e6 check on two values let a lot of garbage
        through. Check every field and put physical bounds on yaw/pitch.
        """
        for v in unpacked:
            if not (abs(v) < 1e6):       # also rejects NaN and inf
                return False
        # Gimbal pitch is mechanically limited; anything outside +/-2 rad is garbage.
        if abs(unpacked[1]) > 2.0:
            return False
        return True

    def _publish_status(self, unpacked):
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

    def _drain_rx_legacy(self):
        """Original raw 40-byte packets, no header/CRC. Kept for current firmware."""
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
                    if self._sanity_ok(unpacked):
                        self._publish_status(unpacked)
                    else:
                        if self._enable_ros_logs:
                            self.get_logger().warn(
                                "RX sanity check failed (garbage values) — flushing buffer")
                        self._ser.reset_input_buffer()
                except struct.error as e:
                    if self._enable_ros_logs:
                        self.get_logger().warn(f"RX unpack error: {e}")
                    self._ser.reset_input_buffer()

    def _drain_rx_framed(self):
        """Framed packets: header + payload + CRC8. Self-resyncing."""
        n = self._ser.in_waiting
        if n > 0:
            self._rx_buf += self._ser.read(n)

        # Cap buffer growth (link flooding / persistent desync).
        if len(self._rx_buf) > RX_BUFFER_CAP:
            del self._rx_buf[:-RX_FRAME_SIZE]

        latest = None
        while True:
            idx = self._rx_buf.find(RX_FRAME_HEADER)
            if idx < 0:
                # No header: keep only the last byte (could be a split header).
                del self._rx_buf[:-1]
                break
            if idx > 0:
                del self._rx_buf[:idx]
            if len(self._rx_buf) < RX_FRAME_SIZE:
                break  # incomplete frame — wait for more bytes

            payload = bytes(self._rx_buf[2:2 + RX_PACKET_SIZE])
            rx_crc = self._rx_buf[2 + RX_PACKET_SIZE]
            if crc8(payload) == rx_crc:
                unpacked = struct.unpack(RX_STRUCT_FORMAT, payload)
                if self._sanity_ok(unpacked):
                    latest = unpacked   # keep only the newest valid frame
                del self._rx_buf[:RX_FRAME_SIZE]
            else:
                # Bad CRC: drop one byte and rescan (auto-resync).
                del self._rx_buf[:1]

        if latest is not None:
            self._publish_status(latest)


    def destroy_node(self):
        if self._ser is not None and self._ser.is_open:
            self._ser.close()
            if self._enable_ros_logs:
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
