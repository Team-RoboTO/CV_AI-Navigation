#!/usr/bin/env python3
"""
game_state_manager.py

Versione robusta:
- legge direttamente /micro_status
- usa 3 subscription a /micro_status con QoS diversi:
  1. default
  2. RELIABLE
  3. BEST_EFFORT

Campi micro_status:
  data[4] = team/color: 0 red, 1 blue
  data[5] = game_progress: 4 = match started
  data[6] = HP
"""

import math
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool, Float32MultiArray, Int32, String


LAB_WAYPOINTS_FILE = "/root/nav2_ws/src/nav2_new/config/arena_waypoints_lab.yaml"


class GameStateManager(Node):
    def __init__(self):
        super().__init__("game_state_manager")

        self.declare_parameter("waypoints_file", LAB_WAYPOINTS_FILE)
        self.declare_parameter("default_team", "red")
        self.declare_parameter("use_team_topic", True)

        self.declare_parameter("wait_strategy", "wait_at_spawn")
        self.declare_parameter("approach_strategy", "percorso")
        self.declare_parameter("loop_strategy", "center_free_strategy")
        self.declare_parameter("retreat_strategy", "retreat_to_spawn")

        self.declare_parameter("retreat_health_threshold", -1.0)
        self.declare_parameter("retreat_wait_sec", 4.0)
        self.declare_parameter("allow_repeated_retreats", True)
        self.declare_parameter("retreat_cooldown_sec", 8.0)

        self.declare_parameter("return_requires_health_recovered", False)
        self.declare_parameter("health_recover_margin", 5.0)

        self.declare_parameter("strategy_republish_sec", 3.0)
        self.declare_parameter("tick_hz", 5.0)
        self.declare_parameter("reset_on_match_end", True)

        self.declare_parameter("use_game_progress_topic", True)
        self.declare_parameter("game_progress_topic", "/game_progress")
        self.declare_parameter("in_match_progress_value", 4)

        self.declare_parameter("micro_status_topic", "/micro_status")
        self.declare_parameter("use_micro_status_progress", True)
        self.declare_parameter("use_micro_status_team", True)
        self.declare_parameter("use_micro_status_health", True)

        self.declare_parameter("micro_team_index", 4)
        self.declare_parameter("micro_game_progress_index", 5)
        self.declare_parameter("micro_health_index", 6)

        self.declare_parameter("reset_when_progress_not_in_match", False)
        self.declare_parameter("stop_on_match_started_false", False)

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

        if retreat_param >= 0.0:
            self.retreat_th = retreat_param
        else:
            self.retreat_th = float(health_cfg.get("retreat_threshold", 30.0))

        self.retreat_wait_sec = float(self.get_parameter("retreat_wait_sec").value)
        self.allow_repeated_retreats = bool(
            self.get_parameter("allow_repeated_retreats").value
        )
        self.retreat_cooldown_sec = float(
            self.get_parameter("retreat_cooldown_sec").value
        )
        self.return_requires_health_recovered = bool(
            self.get_parameter("return_requires_health_recovered").value
        )
        self.health_recover_margin = float(
            self.get_parameter("health_recover_margin").value
        )

        self.strategy_republish_sec = float(
            self.get_parameter("strategy_republish_sec").value
        )
        self.reset_on_match_end = bool(self.get_parameter("reset_on_match_end").value)
        tick_hz = float(self.get_parameter("tick_hz").value)

        self.use_game_progress_topic = bool(
            self.get_parameter("use_game_progress_topic").value
        )
        self.game_progress_topic = str(self.get_parameter("game_progress_topic").value)
        self.in_match_progress_value = int(
            self.get_parameter("in_match_progress_value").value
        )

        self.micro_status_topic = str(self.get_parameter("micro_status_topic").value)
        self.use_micro_status_progress = bool(
            self.get_parameter("use_micro_status_progress").value
        )
        self.use_micro_status_team = bool(
            self.get_parameter("use_micro_status_team").value
        )
        self.use_micro_status_health = bool(
            self.get_parameter("use_micro_status_health").value
        )

        self.micro_team_index = int(self.get_parameter("micro_team_index").value)
        self.micro_game_progress_index = int(
            self.get_parameter("micro_game_progress_index").value
        )
        self.micro_health_index = int(self.get_parameter("micro_health_index").value)

        self.reset_when_progress_not_in_match = bool(
            self.get_parameter("reset_when_progress_not_in_match").value
        )
        self.stop_on_match_started_false = bool(
            self.get_parameter("stop_on_match_started_false").value
        )

        self.match_started = False
        self.health = 999

        self.last_game_progress = None
        self.last_micro_progress = None
        self.last_micro_hp = None
        self.last_micro_team = None
        self.last_micro_status_time = None
        self.first_micro_status_logged = False
        self.last_no_micro_warn_time = 0.0

        self.state = "BOOT"
        self.active_strategy = ""
        self.last_strategy_pub = -1e9
        self.last_retreat_time = -1e9
        self.retreat_wait_until = None

        self.pub_strategy = self.create_publisher(String, "/strategy", 10)
        self.pub_state = self.create_publisher(String, "/game_state_manager/state", 10)
        self.pub_restart = self.create_publisher(Bool, "/nav_restart_requested", 10)

        self._subs = []

        if self.use_team_topic:
            self._subs.append(
                self.create_subscription(
                    String,
                    "/team",
                    self._on_team,
                    10,
                )
            )

        self._subs.append(
            self.create_subscription(
                Bool,
                "/match_started",
                self._on_match_started,
                10,
            )
        )

        self._subs.append(
            self.create_subscription(
                Bool,
                "/match_ended",
                self._on_match_ended,
                10,
            )
        )

        self._subs.append(
            self.create_subscription(
                Int32,
                "/health",
                self._on_health,
                10,
            )
        )

        self._subs.append(
            self.create_subscription(
                String,
                "/waypoint_manager/strategy_completed",
                self._on_strategy_completed,
                10,
            )
        )

        self._subs.append(
            self.create_subscription(
                String,
                "/waypoint_manager/strategy_failed",
                self._on_strategy_failed,
                10,
            )
        )

        if self.use_game_progress_topic:
            self._subs.append(
                self.create_subscription(
                    Int32,
                    self.game_progress_topic,
                    self._on_game_progress,
                    10,
                )
            )

        self._create_micro_status_subscriptions()

        self.timer = self.create_timer(1.0 / max(0.5, tick_hz), self._tick)

        self.get_logger().info(
            "game_state_manager ROBUST MICRO MULTI-QOS started. "
            f"team={self.team}, "
            f"wait={self.s_wait}, "
            f"approach={self.s_approach}, "
            f"loop={self.s_loop}, "
            f"retreat={self.s_retreat}, "
            f"retreat_th={self.retreat_th}, "
            f"micro_status={self.micro_status_topic}, "
            f"team_idx={self.micro_team_index}, "
            f"progress_idx={self.micro_game_progress_index}, "
            f"hp_idx={self.micro_health_index}, "
            f"in_match_value={self.in_match_progress_value}, "
            f"subs={len(self._subs)}"
        )

    def _create_micro_status_subscriptions(self):
        if not (
            self.use_micro_status_progress
            or self.use_micro_status_team
            or self.use_micro_status_health
        ):
            self.get_logger().warn("micro_status subscriptions disabled by parameters")
            return

        micro_status_qos_reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        micro_status_qos_best_effort = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscription default ROS2.
        self._subs.append(
            self.create_subscription(
                Float32MultiArray,
                self.micro_status_topic,
                self._on_micro_status,
                10,
            )
        )

        # Subscription for RELIABLE publisher.
        self._subs.append(
            self.create_subscription(
                Float32MultiArray,
                self.micro_status_topic,
                self._on_micro_status,
                micro_status_qos_reliable,
            )
        )

        # Subscription for BEST_EFFORT publisher.
        self._subs.append(
            self.create_subscription(
                Float32MultiArray,
                self.micro_status_topic,
                self._on_micro_status,
                micro_status_qos_best_effort,
            )
        )

        self.get_logger().info(
            f"created 3 /micro_status subscriptions on {self.micro_status_topic}: "
            "default + RELIABLE + BEST_EFFORT"
        )

    def _validate_strategy(self, name: str):
        if name not in self.strategies:
            self.get_logger().error(f'Strategy "{name}" not found in waypoint YAML')
            raise SystemExit(1)

    @staticmethod
    def _finite(x: float) -> bool:
        return math.isfinite(x)

    def _on_team(self, msg: String):
        t = msg.data.strip().lower()

        if t not in ("red", "blue"):
            return

        if t != self.team:
            self.get_logger().info(f"team topic: {self.team} -> {t}")
            self.team = t

            if not self.match_started:
                self._publish_strategy(self.s_wait, force=True)

    def _set_team_from_code(self, team_code: int, source: str):
        if team_code == 0:
            new_team = "red"
        elif team_code == 1:
            new_team = "blue"
        else:
            return

        if new_team != self.team:
            self.get_logger().info(f"team from {source}: {self.team} -> {new_team}")
            self.team = new_team

            if not self.match_started:
                self._publish_strategy(self.s_wait, force=True)

    def _start_match(self, source: str):
        if self.match_started and self.state not in ("BOOT", "WAITING_MATCH"):
            return

        self.get_logger().info(f"MATCH STARTED from {source}")
        self.match_started = True
        self.state = "APPROACHING"
        self.retreat_wait_until = None
        self._publish_strategy(self.s_approach, force=True)

    def _stop_match(self, source: str):
        if not self.match_started:
            return

        self.get_logger().info(f"MATCH STOPPED from {source}")
        self.match_started = False
        self.state = "WAITING_MATCH"
        self.retreat_wait_until = None
        self._publish_strategy(self.s_wait, force=True)

    def _on_match_started(self, msg: Bool):
        if bool(msg.data):
            self._start_match("/match_started=true")
        elif self.stop_on_match_started_false:
            self._stop_match("/match_started=false")

    def _on_match_ended(self, msg: Bool):
        if bool(msg.data) and self.reset_on_match_end:
            self.get_logger().warn("MATCH ENDED/RESET requested")
            self._reset_for_next_match()

    def _on_game_progress(self, msg: Int32):
        progress = int(msg.data)
        self.last_game_progress = progress

        if progress == self.in_match_progress_value:
            self._start_match(
                f"{self.game_progress_topic} == {self.in_match_progress_value}"
            )
        elif self.reset_when_progress_not_in_match:
            self._stop_match(f"{self.game_progress_topic}={progress}")

    def _on_health(self, msg: Int32):
        self.health = int(msg.data)

    def _on_micro_status(self, msg: Float32MultiArray):
        self.last_micro_status_time = time.monotonic()
        n = len(msg.data)

        if not self.first_micro_status_logged:
            self.first_micro_status_logged = True
            self.get_logger().info(
                f"FIRST /micro_status received, len={n}, data={list(msg.data)}"
            )

        if self.use_micro_status_team and 0 <= self.micro_team_index < n:
            raw_team = float(msg.data[self.micro_team_index])

            if self._finite(raw_team):
                team_code = int(round(raw_team))

                if abs(raw_team - float(team_code)) <= 0.25:
                    self.last_micro_team = team_code
                    self._set_team_from_code(
                        team_code,
                        f"{self.micro_status_topic}[{self.micro_team_index}]",
                    )

        if self.use_micro_status_health and 0 <= self.micro_health_index < n:
            raw_hp = float(msg.data[self.micro_health_index])

            if self._finite(raw_hp):
                self.health = int(round(raw_hp))
                self.last_micro_hp = self.health

        if self.use_micro_status_progress and 0 <= self.micro_game_progress_index < n:
            raw_progress = float(msg.data[self.micro_game_progress_index])

            if self._finite(raw_progress):
                progress = int(round(raw_progress))
                self.last_micro_progress = progress

                if abs(raw_progress - float(progress)) <= 0.25:
                    if progress == self.in_match_progress_value:
                        self._start_match(
                            f"{self.micro_status_topic}[{self.micro_game_progress_index}] "
                            f"== {self.in_match_progress_value}"
                        )
                    elif self.reset_when_progress_not_in_match:
                        self._stop_match(
                            f"{self.micro_status_topic}[{self.micro_game_progress_index}]"
                            f"={progress}"
                        )

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

    def _on_strategy_completed(self, msg: String):
        name = msg.data.strip()
        self.get_logger().info(f"completed strategy from waypoint_manager: {name}")

        if name == self.s_approach and self.state in ("APPROACHING", "RETURNING"):
            self.state = "LOOPING"
            self.retreat_wait_until = None
            self._publish_strategy(self.s_loop, force=True)
            self.get_logger().info(
                f'Approach "{self.s_approach}" complete. '
                f'Starting loop "{self.s_loop}".'
            )
            return

        if name == self.s_retreat and self.state == "RETREATING":
            self.state = "RETREAT_WAIT"
            self.retreat_wait_until = time.monotonic() + self.retreat_wait_sec
            self._publish_strategy(self.s_wait, force=True)
            self.get_logger().info(
                f"Retreat complete. Waiting at spawn for {self.retreat_wait_sec:.1f}s."
            )
            return

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

        if self.last_micro_status_time is None:
            if now - self.last_no_micro_warn_time > 5.0:
                self.last_no_micro_warn_time = now
                self.get_logger().warn(
                    "No /micro_status received yet by game_state_manager. "
                    "State will remain default until callback arrives."
                )

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
            wait_done = (
                self.retreat_wait_until is not None
                and now >= self.retreat_wait_until
            )

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

        micro_age = -1.0

        if self.last_micro_status_time is not None:
            micro_age = max(0.0, time.monotonic() - self.last_micro_status_time)

        m = String()
        m.data = (
            f"state={self.state} "
            f"team={self.team} "
            f"hp={self.health} "
            f"match={self.match_started} "
            f"strategy={self.active_strategy} "
            f"retreat_th={self.retreat_th} "
            f"wait_left={wait_left:.1f} "
            f"game_progress={self.last_game_progress} "
            f"micro_progress={self.last_micro_progress} "
            f"micro_hp={self.last_micro_hp} "
            f"micro_team={self.last_micro_team} "
            f"micro_age={micro_age:.2f}"
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