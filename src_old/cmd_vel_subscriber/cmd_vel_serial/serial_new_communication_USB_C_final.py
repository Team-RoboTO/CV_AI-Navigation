import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Bool, Int32
import serial
import struct
import threading
from rclpy.qos import qos_profile_system_default


# =============================================================================
# PACKET DEFINITIONS
# =============================================================================
#
# TX (Jetson -> Micro): 6 floats = 24 bytes
#   [0] Yaw          (from AI)
#   [1] Pitch        (from AI)
#   [2] Shoot        (from AI)
#   [3] NavX         (from NAV, cmd_vel linear.x)
#   [4] NavY         (from NAV, cmd_vel linear.y)
#   [5] NavAngle     (from NAV, cmd_vel angular.z)
#
# RX (Micro -> Jetson): 6 floats = 24 bytes
#   [0] Color        TBD
#   [1] Start        TBD (1.0 = match started, 0.0 = not yet)
#   [2] Health       (current HP)
#   [3] Ammo         Not necessary?? 
#   [4] Center       TBD (1.0 = center of arena reached)
#   [5] Resupply     TBD (1.0 = resupply zone reached)
# =============================================================================

TX_NUM_VALUES = 6
TX_PACKET_SIZE = TX_NUM_VALUES * 4          # 24 bytes
TX_STRUCT_FORMAT = '<' + ('f' * TX_NUM_VALUES)

RX_NUM_VALUES = 6
RX_PACKET_SIZE = RX_NUM_VALUES * 4          # 24 bytes
RX_STRUCT_FORMAT = '<' + ('f' * RX_NUM_VALUES)


# Able or disable the 2 pipelines informations
AI_PIPELINE  = False 
NAV_PIPELINE = True



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
        # Serial port
        # ------------------------------------------------------------------
        # Checking the correct port, run: ls /dev/tty{ACM,USB}*
        # For a stable name use a udev rule:

        # udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct"

        # SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY",
        #   SYMLINK+="robot_micro"


        try:
            self._ser = serial.Serial(
                port="/dev/ttyUSB0",        # Change to /dev/ttyACM0 or /dev/robot_micro
                baudrate=500000,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,  # Change to PARITY_NONE if micro doesn't use it
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01
            )
            self.get_logger().info("Serial port opened successfully.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit(1)

        # ------------------------------------------------------------------
        # Timer: send TX packet + drain RX at 100 Hz (every 10 ms).
        # Adjust frequency as needed — printing to console slows things down.
        # ------------------------------------------------------------------
        self._timer = self.create_timer(0.01, self._serial_callback)

    # ------------------------------------------------------------------
    # Subscription callbacks  —  just store latest values
    # ------------------------------------------------------------------

    def _ai_callback(self, msg: Twist):
        """Receive detection/turret commands from the AI pipeline."""
        if not AI_PIPELINE:
                return
        with self._lock:
            self._ai_shoot = msg.angular.x   # shoot flag
            self._ai_pitch = msg.angular.y   # pitch
            self._ai_yaw   = -msg.angular.z  # yaw (negated to match micro convention)

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

        # --- Build TX packet ---
        with self._lock:
            data_to_send = [
                self._ai_yaw,
                self._ai_pitch,
                self._ai_shoot,
                self._nav_x,
                self._nav_y,
                self._nav_angle,
            ]

        tx_bytes = struct.pack(TX_STRUCT_FORMAT, *data_to_send)
        self._ser.write(tx_bytes)
        self.get_logger().debug(
            f"[TX] yaw={data_to_send[0]:.2f}  pitch={data_to_send[1]:.2f}  "
            f"shoot={data_to_send[2]:.0f}  "
            f"nav=({data_to_send[3]:.2f}, {data_to_send[4]:.2f}, {data_to_send[5]:.2f})"
        )

        # --- Drain RX buffer ---
        if self._ser.in_waiting >= RX_PACKET_SIZE:
            rx_bytes = self._ser.read(RX_PACKET_SIZE)

            if len(rx_bytes) == RX_PACKET_SIZE:
                try:
                    unpacked = struct.unpack(RX_STRUCT_FORMAT, rx_bytes)
                    # unpacked = (Color, Start, Health, Ammo, Center, Resupply)

                    self.get_logger().debug(
                        f"[RX] color={unpacked[0]:.0f}  start={unpacked[1]:.0f}  "
                        f"health={unpacked[2]:.1f}  ammo={unpacked[3]:.1f}  "
                        f"center={unpacked[4]:.0f}  resupply={unpacked[5]:.0f}"
                    )

                    # Publish all micro data on /micro_status
                    msg = Float32MultiArray()
                    msg.data = list(unpacked)
                    self._micro_status_pub.publish(msg)

                except struct.error as e:
                    self.get_logger().warn(f"RX unpack error: {e}")


    def destroy_node(self):
        if self._ser.is_open:
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