"""
game_state_manager.py

LAB VERSION.

Match FSM driven by micro_status_adapter topics.

Inputs:
  /team std_msgs/String                       red | blue
  /match_started std_msgs/Bool                true only when referee game_progress == IN_MATCH
  /match_ended std_msgs/Bool                  true on MATCH_SETTLING / end transition
  /health std_msgs/Int32                      HP
  /center_status std_msgs/Int32               0 free, 1 ours, 2 enemy
  /waypoint_manager/strategy_completed std_msgs/String
  /waypoint_manager/strategy_failed std_msgs/String

Outputs:
  /strategy std_msgs/String
  /game_state_manager/state std_msgs/String
  /nav_restart_requested std_msgs/Bool

Lab behavior:
  - Before match start: wait_at_spawn.
  - Match start: rush_strategy. Set this to "rush_center" or "percorso" in YAML params.
  - Low HP: retreat_to_spawn.
  - After retreat completes: wait at spawn N seconds, then after_retreat.
  - When rush_strategy / after_retreat completes: choose center strategy from center_status.
  - Center free: center_free_strategy.
  - Center ours: hold_center_diagonal.
  - Center enemy: enemy_center_strategy.
  - Match end: reset to wait_at_spawn and publish /nav_restart_requested.
"""

import time
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


class GameStateManager(Node):
    def __init__(self):
        super().__init__("game_state_manager")

        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("default_team", "red")
        self.declare_parameter("use_team_topic", True)

        self.declare_parameter("wait_strategy", "wait_at_spawn")
        self.declare_parameter("rush_strategy", "rush_center")
        self.declare_parameter("retreat_strategy", "retreat_to_spawn")
        self.declare_parameter("after_retreat_strategy", "after_retreat")

        self.declare_parameter("center_free_strategy", "center_free_strategy")
        self.declare_parameter("center_ours_strategy", "hold_center_diagonal")
        self.declare_parameter("center_enemy_strategy", "enemy_center_strategy")

        self.declare_parameter("retreat_health_threshold", -1.0)
        self.declare_parameter("retreat_wait_sec", 4.0)
        self.declare_parameter("allow_repeated_retreats", True)
        self.declare_parameter("retreat_cooldown_sec", 8.0)

        self.declare_parameter("strategy_republish_sec", 3.0)
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("reset_on_match_end", True)

        wp_file = self.get_parameter("waypoints_file").value
        if not wp_file:
            self.get_logger().error("waypoints_file is required")
            raise SystemExit(1)

        with open(wp_file, "r") as f:
            cfg = yaml.safe_load(f)

        self.strategies = cfg.get("strategies", {})
        h = cfg.get("health", {})

        retreat_param = float(self.get_parameter("retreat_health_threshold").value)
        self.retreat_th = retreat_param if retreat_param >= 0.0 else float(h.get("retreat_threshold", 30))

        self.team = str(self.get_parameter("default_team").value).lower()
        self.use_team_topic = bool(self.get_parameter("use_team_topic").value)

        self.s_wait = self.get_parameter("wait_strategy").value
        self.s_rush = self.get_parameter("rush_strategy").value
        self.s_retreat = self.get_parameter("retreat_strategy").value
        self.s_after_retreat = self.get_parameter("after_retreat_strategy").value
        self.s_center_free = self.get_parameter("center_free_strategy").value
        self.s_center_ours = self.get_parameter("center_ours_strategy").value
        self.s_center_enemy = self.get_parameter("center_enemy_strategy").value

        self._validate_strategy(self.s_wait)
        self._validate_strategy(self.s_rush)
        self._validate_strategy(self.s_retreat)
        self._validate_strategy(self.s_after_retreat)
        self._validate_strategy(self.s_center_free)
        self._validate_strategy(self.s_center_ours)
        self._validate_strategy(self.s_center_enemy)

        self.retreat_wait_sec = float(self.get_parameter("retreat_wait_sec").value)
        self.allow_repeated_retreats = bool(self.get_parameter("allow_repeated_retreats").value)
        self.retreat_cooldown_sec = float(self.get_parameter("retreat_cooldown_sec").value)
        self.strategy_republish_sec = float(self.get_parameter("strategy_republish_sec").value)
        self.reset_on_match_end = bool(self.get_parameter("reset_on_match_end").value)
        tick_hz = float(self.get_parameter("tick_hz").value)

        self.match_started = False
        self.health = 999
        self.center_status = 0

        self.active_strategy = ""
        self.state = "BOOT"
        self.last_strategy_pub = -1e9
        self.last_completed_strategy = ""
        self.last_retreat_time = -1e9
        self.retreat_wait_until = None
        self._match_was_started = False

        self.pub_strategy = self.create_publisher(String, "/strategy", 10)
        self.pub_state = self.create_publisher(String, "/game_state_manager/state", 10)
        self.pub_restart = self.create_publisher(Bool, "/nav_restart_requested", 10)

        if self.use_team_topic:
            self.create_subscription(String, "/team", self._on_team, 10)

        self.create_subscription(Bool, "/match_started", self._on_match_started, 10)
        self.create_subscription(Bool, "/match_ended", self._on_match_ended, 10)
        self.create_subscription(Int32, "/health", self._on_health, 10)
        self.create_subscription(Int32, "/center_status", self._on_center_status, 10)
        self.create_subscription(String, "/waypoint_manager/strategy_completed", self._on_strategy_completed, 10)
        self.create_subscription(String, "/waypoint_manager/strategy_failed", self._on_strategy_failed, 10)

        self.create_timer(1.0 / max(0.5, tick_hz), self._tick)

        self.get_logger().info(
            f"game_state_manager LAB started. team={self.team}, retreat_th={self.retreat_th}, "
            f"wait={self.s_wait}, rush={self.s_rush}, after_retreat={self.s_after_retreat}"
        )

    def _validate_strategy(self, name: str):
        if name not in self.strategies:
            self.get_logger().error(f'Strategy "{name}" not found in lab YAML')
            raise SystemExit(1)

    def _on_team(self, msg: String):
        t = msg.data.strip().lower()

        if t in ("red", "blue") and t != self.team:
            self.get_logger().info(f"team: {self.team} -> {t}")
            self.team = t

            if not self.match_started:
                self._publish_strategy(self.s_wait, force=True)

    def _on_match_started(self, msg: Bool):
        new = bool(msg.data)

        if new and not self.match_started:
            self.get_logger().info("MATCH STARTED from micro")
            self.state = "RUSHING"
            self.retreat_wait_until = None
            self._publish_strategy(self.s_rush, force=True)

        elif not new and self.match_started:
            self.get_logger().info("MATCH no longer in progress")

        self.match_started = new
        self._match_was_started = self._match_was_started or new

    def _on_match_ended(self, msg: Bool):
        if msg.data and self.reset_on_match_end:
            self.get_logger().warn("MATCH ENDED/RESET requested by micro")
            self._reset_for_next_match()

    def _reset_for_next_match(self):
        self.match_started = False
        self._match_was_started = False
        self.state = "WAITING_MATCH"
        self.retreat_wait_until = None
        self.active_strategy = ""
        self.last_strategy_pub = -1e9

        self._publish_strategy(self.s_wait, force=True)

        b = Bool()
        b.data = True
        self.pub_restart.publish(b)

    def _on_health(self, msg: Int32):
        self.health = int(msg.data)

    def _on_center_status(self, msg: Int32):
        old = self.center_status
        self.center_status = int(msg.data)

        if old != self.center_status:
            self.get_logger().info(f"center_status: {old} -> {self.center_status}")

            if self.state in ("CENTER_FREE", "CENTER_OURS", "CENTER_ENEMY") and self.match_started:
                self._select_center_strategy(force=True)

    def _on_strategy_completed(self, msg: String):
        name = msg.data.strip()
        self.last_completed_strategy = name
        self.get_logger().info(f"completed strategy from waypoint_manager: {name}")

        if name == self.s_retreat and self.state == "RETREATING":
            self.state = "RETREAT_WAIT"
            self.retreat_wait_until = time.monotonic() + self.retreat_wait_sec
            self._publish_strategy(self.s_wait, force=True)
            self.get_logger().info(
                f"Retreat complete. Waiting at spawn for {self.retreat_wait_sec:.1f}s"
            )

        elif name == self.s_rush and self.state in ("RUSHING", "RETURNING_CENTER"):
            self._select_center_strategy(force=True)

        elif name == self.s_after_retreat and self.state == "RETURNING_CENTER":
            self._select_center_strategy(force=True)

    def _on_strategy_failed(self, msg: String):
        self.get_logger().warn(f"waypoint strategy failed: {msg.data}")

    def _publish_strategy(self, strategy: str, force: bool = False):
        now = time.monotonic()

        if (
            not force
            and self.active_strategy == strategy
            and now - self.last_strategy_pub < self.strategy_republish_sec
        ):
            return

        self.active_strategy = strategy
        self.last_strategy_pub = now

        m = String()
        m.data = strategy
        self.pub_strategy.publish(m)

        self.get_logger().info(f"-> /strategy: {strategy}")

    def _select_center_strategy(self, force=False):
        if self.center_status == 2:
            self.state = "CENTER_ENEMY"
            self._publish_strategy(self.s_center_enemy, force=force)

        elif self.center_status == 1:
            self.state = "CENTER_OURS"
            self._publish_strategy(self.s_center_ours, force=force)

        else:
            self.state = "CENTER_FREE"
            self._publish_strategy(self.s_center_free, force=force)

    def _low_hp_should_retreat(self):
        if self.health >= self.retreat_th:
            return False

        if self.state in ("RETREATING", "RETREAT_WAIT"):
            return False

        if not self.allow_repeated_retreats:
            return self.last_retreat_time < -1e8

        return (time.monotonic() - self.last_retreat_time) > self.retreat_cooldown_sec

    def _tick(self):
        now = time.monotonic()

        if not self.match_started:
            if self.state != "WAITING_MATCH":
                self.state = "WAITING_MATCH"
                self._publish_strategy(self.s_wait, force=True)
            else:
                self._publish_strategy(self.s_wait)

            self._publish_state()
            return

        if self._low_hp_should_retreat():
            self.state = "RETREATING"
            self.last_retreat_time = now
            self.retreat_wait_until = None
            self._publish_strategy(self.s_retreat, force=True)
            self._publish_state()
            return

        if self.state == "RETREAT_WAIT":
            if self.retreat_wait_until is not None and now >= self.retreat_wait_until:
                self.state = "RETURNING_CENTER"
                self._publish_strategy(self.s_after_retreat, force=True)
            else:
                self._publish_strategy(self.s_wait)

            self._publish_state()
            return

        if self.state in ("BOOT", "WAITING_MATCH"):
            self.state = "RUSHING"
            self._publish_strategy(self.s_rush, force=True)

        elif self.state == "RUSHING":
            self._publish_strategy(self.s_rush)

        elif self.state == "RETURNING_CENTER":
            self._publish_strategy(self.s_after_retreat)

        elif self.state in ("CENTER_FREE", "CENTER_OURS", "CENTER_ENEMY"):
            self._select_center_strategy(force=False)

        self._publish_state()

    def _publish_state(self):
        m = String()
        m.data = (
            f"state={self.state} team={self.team} hp={self.health} "
            f"match={self.match_started} center={self.center_status} strategy={self.active_strategy}"
        )
        self.pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GameStateManager()
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
