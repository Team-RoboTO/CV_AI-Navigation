#!/usr/bin/env python3
"""
game_state_manager.py

LAB / APPROACH + LOOP + RETREAT VERSION.

Desired behavior:

  1. Before match:
       wait_at_spawn

  2. Match starts:
       percorso

  3. percorso completed:
       loop_strategy
       The loop itself is handled by waypoint_manager via loop_strategies.

  4. HP below threshold, only while in percorso/loop:
       retreat_to_spawn

  5. retreat_to_spawn completed:
       wait_at_spawn for retreat_wait_sec seconds

  6. Wait done:
       percorso again
       then loop_strategy again

So the cycle is:

  wait_at_spawn
    -> percorso
    -> loop_strategy -> loop_strategy -> loop_strategy ...
    -> low HP
    -> retreat_to_spawn
    -> wait_at_spawn
    -> percorso
    -> loop_strategy -> ...

Inputs:
  /team                                      std_msgs/String
  /match_started                             std_msgs/Bool
  /match_ended                               std_msgs/Bool
  /health                                    std_msgs/Int32
  /waypoint_manager/strategy_completed       std_msgs/String
  /waypoint_manager/strategy_failed          std_msgs/String

Outputs:
  /strategy                                  std_msgs/String
  /game_state_manager/state                  std_msgs/String
  /nav_restart_requested                     std_msgs/Bool

Notes:
  - center_status is intentionally ignored here.
  - Only HP can interrupt the loop strategy.
  - waypoint_manager must have loop_strategies containing loop_strategy.
"""

import time
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


class GameStateManager(Node):
    def __init__(self):
        super().__init__("game_state_manager")

        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("default_team", "red")
        self.declare_parameter("use_team_topic", True)

        # Strategy names. Change loop_strategy to your real loop strategy name
        # if it is not center_free_strategy, for example hold_center_diagonal.
        self.declare_parameter("wait_strategy", "wait_at_spawn")
        self.declare_parameter("approach_strategy", "percorso")
        self.declare_parameter("loop_strategy", "center_free_strategy")
        self.declare_parameter("retreat_strategy", "retreat_to_spawn")

        # Low HP behavior.
        # If retreat_health_threshold < 0, read health.retreat_threshold from arena_waypoints_lab.yaml.
        self.declare_parameter("retreat_health_threshold", -1.0)
        self.declare_parameter("retreat_wait_sec", 4.0)
        self.declare_parameter("allow_repeated_retreats", True)
        self.declare_parameter("retreat_cooldown_sec", 8.0)

        # Usually false for lab testing: after retreat_wait_sec, go back through
        # percorso even if the health topic has not refreshed yet. The cooldown
        # prevents immediate re-retreat spam.
        self.declare_parameter("return_requires_health_recovered", False)
        self.declare_parameter("health_recover_margin", 5.0)

        # Publication behavior.
        self.declare_parameter("strategy_republish_sec", 3.0)
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("reset_on_match_end", True)

        wp_file = str(self.get_parameter("waypoints_file").value)
        if not wp_file:
            self.get_logger().error("waypoints_file is required")
            raise SystemExit(1)

        with open(wp_file, "r") as f:
            cfg = yaml.safe_load(f) or {}

        self.strategies = cfg.get("strategies", {}) or {}
        health_cfg = cfg.get("health", {}) or {}

        self.team = str(self.get_parameter("default_team").value).strip().lower()
        self.use_team_topic = bool(self.get_parameter("use_team_topic").value)

        self.s_wait = str(self.get_parameter("wait_strategy").value)
        self.s_approach = str(self.get_parameter("approach_strategy").value)
        self.s_loop = str(self.get_parameter("loop_strategy").value)
        self.s_retreat = str(self.get_parameter("retreat_strategy").value)

        self._validate_strategy(self.s_wait)
        self._validate_strategy(self.s_approach)
        self._validate_strategy(self.s_loop)
        self._validate_strategy(self.s_retreat)

        retreat_param = float(self.get_parameter("retreat_health_threshold").value)
        self.retreat_th = (
            retreat_param if retreat_param >= 0.0
            else float(health_cfg.get("retreat_threshold", 30.0))
        )

        self.retreat_wait_sec = float(self.get_parameter("retreat_wait_sec").value)
        self.allow_repeated_retreats = bool(self.get_parameter("allow_repeated_retreats").value)
        self.retreat_cooldown_sec = float(self.get_parameter("retreat_cooldown_sec").value)
        self.return_requires_health_recovered = bool(
            self.get_parameter("return_requires_health_recovered").value
        )
        self.health_recover_margin = float(self.get_parameter("health_recover_margin").value)
        self.strategy_republish_sec = float(self.get_parameter("strategy_republish_sec").value)
        self.reset_on_match_end = bool(self.get_parameter("reset_on_match_end").value)
        tick_hz = float(self.get_parameter("tick_hz").value)

        self.match_started = False
        self.health = 999

        # States:
        #   WAITING_MATCH, APPROACHING, LOOPING, RETREATING, RETREAT_WAIT, RETURNING
        self.state = "BOOT"
        self.active_strategy = ""
        self.last_strategy_pub = -1e9
        self.last_retreat_time = -1e9
        self.retreat_wait_until = None

        self.pub_strategy = self.create_publisher(String, "/strategy", 10)
        self.pub_state = self.create_publisher(String, "/game_state_manager/state", 10)
        self.pub_restart = self.create_publisher(Bool, "/nav_restart_requested", 10)

        if self.use_team_topic:
            self.create_subscription(String, "/team", self._on_team, 10)

        self.create_subscription(Bool, "/match_started", self._on_match_started, 10)
        self.create_subscription(Bool, "/match_ended", self._on_match_ended, 10)
        self.create_subscription(Int32, "/health", self._on_health, 10)
        self.create_subscription(String, "/waypoint_manager/strategy_completed", self._on_strategy_completed, 10)
        self.create_subscription(String, "/waypoint_manager/strategy_failed", self._on_strategy_failed, 10)

        self.create_timer(1.0 / max(0.5, tick_hz), self._tick)

        self.get_logger().info(
            "game_state_manager LAB approach-loop-retreat started. "
            f"team={self.team}, wait={self.s_wait}, approach={self.s_approach}, "
            f"loop={self.s_loop}, retreat={self.s_retreat}, "
            f"retreat_th={self.retreat_th}, retreat_wait={self.retreat_wait_sec:.1f}s"
        )

    def _validate_strategy(self, name: str):
        if name not in self.strategies:
            self.get_logger().error(f'Strategy "{name}" not found in lab YAML')
            raise SystemExit(1)

    def _on_team(self, msg: String):
        t = msg.data.strip().lower()
        if t not in ("red", "blue"):
            return

        if t != self.team:
            self.get_logger().info(f"team: {self.team} -> {t}")
            self.team = t

            # Only force refresh before match. During match team should not change.
            if not self.match_started:
                self._publish_strategy(self.s_wait, force=True)

    def _on_match_started(self, msg: Bool):
        new = bool(msg.data)

        if new and not self.match_started:
            self.get_logger().info("MATCH STARTED from micro")
            self.match_started = True
            self.state = "APPROACHING"
            self.retreat_wait_until = None
            self._publish_strategy(self.s_approach, force=True)
            return

        if not new and self.match_started:
            self.get_logger().info("MATCH no longer in progress")
            self.match_started = False
            self.state = "WAITING_MATCH"
            self.retreat_wait_until = None
            self._publish_strategy(self.s_wait, force=True)
            return

        self.match_started = new

    def _on_match_ended(self, msg: Bool):
        if bool(msg.data) and self.reset_on_match_end:
            self.get_logger().warn("MATCH ENDED/RESET requested by micro")
            self._reset_for_next_match()

    def _reset_for_next_match(self):
        self.match_started = False
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

    def _on_strategy_completed(self, msg: String):
        name = msg.data.strip()
        self.get_logger().info(f"completed strategy from waypoint_manager: {name}")

        if name == self.s_approach and self.state in ("APPROACHING", "RETURNING"):
            self.state = "LOOPING"
            self.retreat_wait_until = None
            self._publish_strategy(self.s_loop, force=True)
            self.get_logger().info(
                f'Approach strategy "{self.s_approach}" complete. Starting loop strategy "{self.s_loop}".'
            )
            return

        if name == self.s_retreat and self.state == "RETREATING":
            self.state = "RETREAT_WAIT"
            self.retreat_wait_until = time.monotonic() + self.retreat_wait_sec
            self._publish_strategy(self.s_wait, force=True)
            self.get_logger().info(
                f'Retreat complete. Waiting at spawn with "{self.s_wait}" for '
                f"{self.retreat_wait_sec:.1f}s, then returning through {self.s_approach}."
            )
            return

        # loop_strategy completion is normally handled by waypoint_manager looping it.
        # wait_strategy completion during RETREAT_WAIT is ignored; timer controls the wait.

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

    def _low_hp_should_retreat(self) -> bool:
        if not self.match_started:
            return False

        if self.health >= self.retreat_th:
            return False

        # Do not retreat while already retreating or waiting at spawn.
        if self.state in ("RETREATING", "RETREAT_WAIT", "WAITING_MATCH", "BOOT"):
            return False

        if not self.allow_repeated_retreats:
            return self.last_retreat_time < -1e8

        return (time.monotonic() - self.last_retreat_time) > self.retreat_cooldown_sec

    def _health_recovered_enough(self) -> bool:
        if not self.return_requires_health_recovered:
            return True
        return self.health >= (self.retreat_th + self.health_recover_margin)

    def _tick(self):
        now = time.monotonic()

        if not self.match_started:
            if self.state != "WAITING_MATCH":
                self.state = "WAITING_MATCH"
                self.retreat_wait_until = None
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
            wait_done = self.retreat_wait_until is not None and now >= self.retreat_wait_until
            if wait_done and self._health_recovered_enough():
                self.state = "RETURNING"
                self.retreat_wait_until = None
                self._publish_strategy(self.s_approach, force=True)
            else:
                self._publish_strategy(self.s_wait)
            self._publish_state()
            return

        if self.state in ("BOOT", "WAITING_MATCH"):
            self.state = "APPROACHING"
            self._publish_strategy(self.s_approach, force=True)
        elif self.state == "APPROACHING":
            self._publish_strategy(self.s_approach)
        elif self.state == "RETURNING":
            self._publish_strategy(self.s_approach)
        elif self.state == "LOOPING":
            self._publish_strategy(self.s_loop)
        elif self.state == "RETREATING":
            self._publish_strategy(self.s_retreat)
        else:
            self.get_logger().warn(f"Unknown state={self.state}; forcing APPROACHING")
            self.state = "APPROACHING"
            self._publish_strategy(self.s_approach, force=True)

        self._publish_state()

    def _publish_state(self):
        wait_left = 0.0
        if self.retreat_wait_until is not None:
            wait_left = max(0.0, self.retreat_wait_until - time.monotonic())

        m = String()
        m.data = (
            f"state={self.state} team={self.team} hp={self.health} "
            f"match={self.match_started} strategy={self.active_strategy} "
            f"retreat_th={self.retreat_th} wait_left={wait_left:.1f}"
        )
        self.pub_state.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GameStateManager()
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
