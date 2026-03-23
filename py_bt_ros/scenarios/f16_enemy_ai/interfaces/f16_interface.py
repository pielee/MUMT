import logging
import math
import time
from dataclasses import dataclass, field

from modules.utils import optional_import

LOGGER = logging.getLogger(__name__)


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def _wrap_angle_deg(angle_deg):
    return ((angle_deg + 180.0) % 360.0) - 180.0


def _is_finite_number(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _safe_float(value, default=0.0):
    if _is_finite_number(value):
        return float(value)
    return float(default)


@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class ControlCommand:
    throttle: float = 0.0
    elevator: float = 0.0
    aileron: float = 0.0
    rudder: float = 0.0
    wheel_brake: float = 0.0
    parking_brake: bool = False
    engine_start: bool = True
    gear_down: bool = True
    control_enabled: bool = True
    source_node: str = ""
    source_branch: str = ""
    command_profile: str = ""
    safety_mode_reason: str = ""


@dataclass
class AircraftState:
    actor_id: str = ""
    position_m: Vector3 = field(default_factory=Vector3)
    velocity_mps: Vector3 = field(default_factory=Vector3)
    ground_speed_mps: float = 0.0
    yaw_deg: float = 0.0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    airspeed_mps: float = 0.0
    altitude_m: float = 0.0
    altitude_world_m: float = 0.0
    altitude_agl_m: float = 0.0
    vertical_speed_mps: float = 0.0
    angle_of_attack_deg: float = 0.0
    timestamp_s: float = 0.0
    state_age_s: float = 0.0
    on_ground: bool = True
    engine_running: bool = False
    parking_brake: bool = False
    wheel_brake: float = 0.0
    gear_down: bool = True
    control_enabled: bool = True
    valid: bool = True
    status_message: str = ""
    state_source: str = ""

    @property
    def bank_deg(self):
        return self.roll_deg


class F16Interface:
    def __init__(self, config, agent=None):
        self.config = config or {}
        self.agent = agent
        self.cfg = self.config.get("f16_interface", {})
        self.mock_cfg = self.cfg.get("mock_backend", {})
        self.backend_mode = str(self.cfg.get("backend_mode", "mock")).lower()
        self.strict_external_adapter = bool(self.cfg.get("strict_external_adapter", True))
        self.strict_state_validation = bool(self.cfg.get("strict_state_validation", True))
        self.require_actor_ids_in_external = bool(
            self.cfg.get("require_actor_ids_in_external", True)
        )
        self.require_command_ack = bool(self.cfg.get("require_command_ack", True))
        self.max_state_age_s = float(self.cfg.get("max_state_age_s", 0.5))
        self.altitude_consistency_tolerance_m = float(
            self.cfg.get("altitude_consistency_tolerance_m", 5.0)
        )
        self.debug_logging = bool(self.cfg.get("debug_logging", True))
        self.command_logging = bool(self.cfg.get("command_logging", True))
        self.state_logging = bool(self.cfg.get("state_logging", True))
        self.state_log_interval_s = float(self.cfg.get("state_log_interval_s", 0.5))
        self.movement_check_interval_s = float(self.cfg.get("movement_check_interval_s", 1.0))

        self.ego_actor_id = str(
            self.cfg.get("ego_actor_id") or getattr(self.agent, "agent_id", "") or ""
        ).strip()
        self.enemy_actor_id = str(self.cfg.get("enemy_actor_id") or "").strip()

        self._last_state_log_s = 0.0
        self._last_motion_warning_s = 0.0
        self._command_sequence = 0
        self._adapter = None
        self._adapter_name = None
        self._fetch_state_hook = None
        self._command_hook = None
        self._last_command = ControlCommand()
        self._command_context = {"node": "", "branch": ""}

        self._ego_state = self._coerce_state(
            self.cfg.get("ego_initial_state", {}),
            entity_name="ego",
            expected_actor_id=self.ego_actor_id,
            default_on_ground=True,
            allow_incomplete=(self.backend_mode != "external"),
        )
        self._enemy_state = self._coerce_state(
            self.cfg.get("enemy_initial_state", {}),
            entity_name="enemy",
            expected_actor_id=self.enemy_actor_id,
            default_on_ground=False,
            allow_incomplete=(self.backend_mode != "external"),
        )

        self._auto_install_backend_adapter()

        if self.backend_mode == "external":
            self._ego_state.valid = False
            self._ego_state.status_message = "Waiting for external ego telemetry"
            self._enemy_state.valid = False
            self._enemy_state.status_message = "Waiting for external enemy telemetry"

        self._log(
            "INFO",
            (
                f"F16Interface initialized backend={self.backend_mode} "
                f"ego_actor_id={self.ego_actor_id or 'UNBOUND'} "
                f"enemy_actor_id={self.enemy_actor_id or 'UNBOUND'}"
            ),
        )

    def install_backend_adapter(
        self,
        fetch_state_hook=None,
        command_hook=None,
        ego_actor_id=None,
        enemy_actor_id=None,
        adapter=None,
        adapter_name=None,
    ):
        if adapter is not None:
            self._adapter = adapter
            if fetch_state_hook is None:
                fetch_state_hook = getattr(adapter, "fetch_state", None)
            if command_hook is None:
                command_hook = getattr(adapter, "send_command", None)
            if ego_actor_id is None:
                ego_actor_id = getattr(adapter, "ego_actor_id", None)
            if enemy_actor_id is None:
                enemy_actor_id = getattr(adapter, "enemy_actor_id", None)

        if fetch_state_hook is not None:
            self._fetch_state_hook = fetch_state_hook
        if command_hook is not None:
            self._command_hook = command_hook
        if ego_actor_id is not None:
            self.ego_actor_id = str(ego_actor_id).strip()
        if enemy_actor_id is not None:
            self.enemy_actor_id = str(enemy_actor_id).strip()

        self._adapter_name = (
            adapter_name
            or getattr(adapter, "__class__", type("UnknownAdapter", (), {})).__name__
            or "direct_hooks"
        )
        self._log(
            "INFO",
            (
                f"Installed external adapter name={self._adapter_name} "
                f"ego_actor_id={self.ego_actor_id or 'UNBOUND'} "
                f"enemy_actor_id={self.enemy_actor_id or 'UNBOUND'} "
                f"fetch_hook={'yes' if self._fetch_state_hook else 'no'} "
                f"command_hook={'yes' if self._command_hook else 'no'}"
            ),
        )

    def tick(self, dt):
        if self.backend_mode == "mock":
            self._advance_mock_backend(dt)
        else:
            self._ensure_external_adapter_ready()
            self._refresh_external_state()

        self._maybe_log_state_snapshots()
        self._maybe_warn_ground_motion_blockers()

    def get_ego_state(self):
        return self._copy_state(self._ego_state)

    def get_enemy_state(self):
        return self._copy_state(self._enemy_state)

    def get_command_target_id(self):
        return self.ego_actor_id or "UNBOUND"

    def describe_ego_binding(self):
        return (
            f"ego_actor_id={self.ego_actor_id or 'UNBOUND'} "
            f"adapter={self._adapter_name or 'UNBOUND'}"
        )

    def set_command_context(self, source_node="", source_branch=""):
        self._command_context = {
            "node": str(source_node or "").strip(),
            "branch": str(source_branch or "").strip(),
        }

    def send_ego_command(self, throttle, elevator, aileron, rudder):
        command = ControlCommand(
            throttle=_clamp(float(throttle), 0.0, 1.0),
            elevator=_clamp(float(elevator), -1.0, 1.0),
            aileron=_clamp(float(aileron), -1.0, 1.0),
            rudder=_clamp(float(rudder), -1.0, 1.0),
            source_node=self._command_context.get("node", ""),
            source_branch=self._command_context.get("branch", ""),
        )
        command = self._apply_ground_ops(command)
        command = self._apply_command_limits(command)

        self._last_command = command
        self._command_sequence += 1

        if self.command_logging:
            self._log(
                "INFO",
                (
                    f"send_ego_command seq={self._command_sequence} "
                    f"source={command.source_node or 'UNKNOWN'} "
                    f"branch={command.source_branch or 'unspecified'} "
                    f"profile={command.command_profile or 'default'} "
                    f"safety={command.safety_mode_reason or 'none'} "
                    f"actor_id={self.get_command_target_id()} "
                    f"controls(thr/ele/ail/rud)=({command.throttle:.2f},{command.elevator:.2f},{command.aileron:.2f},{command.rudder:.2f}) "
                    f"receiver(roll,pitch,yaw,thr)=({command.aileron:.2f},{command.elevator:.2f},{command.rudder:.2f},{command.throttle:.2f}) "
                    f"wheel_brake={command.wheel_brake:.2f} "
                    f"parking_brake={command.parking_brake} "
                    f"engine_start={command.engine_start} gear_down={command.gear_down}"
                ),
            )

        if self.backend_mode == "external":
            self._push_external_command(command)

    def close(self):
        adapter = getattr(self, "_adapter", None)
        if adapter is not None and hasattr(adapter, "close"):
            adapter.close()

    def _auto_install_backend_adapter(self):
        adapter_path = self.cfg.get("backend_adapter_class")
        if not adapter_path:
            return

        module_name, class_name = adapter_path.rsplit(".", 1)
        adapter_module = optional_import(module_name)
        if adapter_module is None:
            raise ModuleNotFoundError(
                f"[ERROR] Could not import backend adapter module '{module_name}'."
            )

        adapter_class = getattr(adapter_module, class_name, None)
        if adapter_class is None:
            raise AttributeError(
                f"[ERROR] Backend adapter class '{class_name}' was not found in '{module_name}'."
            )

        adapter = adapter_class(config=self.config, interface=self, agent=self.agent)
        self.install_backend_adapter(adapter=adapter, adapter_name=adapter_path)

    def _apply_ground_ops(self, command):
        ground_cfg = self.cfg.get("ground_ops", {})
        if not ground_cfg.get("enabled", True):
            return command

        command.control_enabled = bool(ground_cfg.get("force_control_enabled", True))
        command.engine_start = bool(ground_cfg.get("auto_engine_start", True))

        throttle_release_threshold = float(
            ground_cfg.get("brake_release_throttle_threshold", 0.05)
        )
        if getattr(self._ego_state, "on_ground", False):
            command.gear_down = bool(ground_cfg.get("keep_gear_down_on_ground", True))
            if command.throttle >= throttle_release_threshold and ground_cfg.get(
                "auto_release_brakes", True
            ):
                command.parking_brake = False
                command.wheel_brake = float(ground_cfg.get("released_wheel_brake", 0.0))
            else:
                command.parking_brake = bool(
                    ground_cfg.get("hold_parking_brake_when_idle", False)
                )
                command.wheel_brake = float(ground_cfg.get("idle_wheel_brake", 0.0))
        else:
            command.parking_brake = False
            command.wheel_brake = 0.0
            command.gear_down = not bool(ground_cfg.get("retract_gear_in_air", False))

        return command

    def _apply_command_limits(self, command):
        limits_root = self.cfg.get("command_limits", {})
        profile_name = self._select_command_profile(command.source_node, command.source_branch)
        limits_cfg = self._merge_command_limit_cfg(limits_root, profile_name)

        safety_reason = self._get_safety_mode_reason(command)
        if safety_reason:
            command = self._build_safe_mode_command(command, safety_reason, limits_root)
            profile_name = command.command_profile or "safe_mode"
            limits_cfg = self._merge_command_limit_cfg(limits_root, profile_name)

        original = (
            command.throttle,
            command.elevator,
            command.aileron,
            command.rudder,
        )

        command.command_profile = profile_name
        command.throttle = _clamp(
            command.throttle,
            float(limits_cfg.get("min_throttle_cmd", 0.60)),
            float(limits_cfg.get("max_throttle_cmd", 0.85)),
        )
        command.elevator = _clamp(
            command.elevator,
            -float(limits_cfg.get("max_pitch_cmd", 0.10)),
            float(limits_cfg.get("max_pitch_cmd", 0.10)),
        )
        command.aileron = _clamp(
            command.aileron,
            -float(limits_cfg.get("max_roll_cmd", 0.15)),
            float(limits_cfg.get("max_roll_cmd", 0.15)),
        )
        command.rudder = _clamp(
            command.rudder,
            -float(limits_cfg.get("max_yaw_cmd", 0.10)),
            float(limits_cfg.get("max_yaw_cmd", 0.10)),
        )

        shaped = (
            command.throttle,
            command.elevator,
            command.aileron,
            command.rudder,
        )
        if any(abs(a - b) > 1e-6 for a, b in zip(original, shaped)):
            self._log(
                "INFO",
                (
                    f"command_limited source={command.source_node or 'UNKNOWN'} "
                    f"branch={command.source_branch or 'unspecified'} profile={profile_name} "
                    f"raw=({original[0]:.2f},{original[1]:.2f},{original[2]:.2f},{original[3]:.2f}) "
                    f"clamped=({shaped[0]:.2f},{shaped[1]:.2f},{shaped[2]:.2f},{shaped[3]:.2f})"
                ),
            )

        return command

    def _select_command_profile(self, source_node, source_branch):
        node_name = str(source_node or "").strip().lower()
        branch_name = str(source_branch or "").strip().lower()

        if node_name == "takeoff" or branch_name in {"takeoff", "ground_roll", "initial_liftoff"}:
            return "takeoff"
        if node_name == "climbtosafealtitude" or branch_name in {
            "runway_hold",
            "safe_climb_hold",
            "limited_turn_to_target",
            "protected_departure",
        }:
            return "climb"
        if node_name == "intercepttarget" or branch_name == "lead_intercept":
            return "intercept"
        if node_name == "getbehindtarget" or branch_name == "rear_positioning":
            return "get_behind"
        if node_name == "followtarget" or branch_name == "rear_follow":
            return "follow"
        if node_name == "reacquiretarget" or branch_name == "last_known_track":
            return "reacquire"
        if node_name == "recover" or branch_name in {
            "stall_unload",
            "low_altitude_escape",
            "pitch_unload",
            "attitude_recovery",
        }:
            return "recover"
        return "default"

    def _is_departure_context(self, command):
        node_name = str(getattr(command, "source_node", "") or "").strip().lower()
        branch_name = str(getattr(command, "source_branch", "") or "").strip().lower()
        if node_name in {"takeoff", "climbtosafealtitude"}:
            return True
        return branch_name in {
            "takeoff",
            "ground_roll",
            "initial_liftoff",
            "protected_departure",
            "runway_hold",
            "safe_climb_hold",
        }

    def _merge_command_limit_cfg(self, limits_root, profile_name):
        merged = dict(limits_root.get("default", {}) or {})
        if profile_name in {"intercept", "get_behind", "follow", "reacquire"}:
            merged.update(limits_root.get("pursuit", {}) or {})
        merged.update(limits_root.get(profile_name, {}) or {})
        return merged

    def _get_safety_mode_reason(self, command):
        if self.backend_mode != "external":
            return ""

        ego = self._ego_state
        target = self._enemy_state
        departure_context = self._is_departure_context(command)

        if not ego.valid:
            return "ego_state_invalid"
        if departure_context:
            return ""

        if not target.valid:
            return "target_state_invalid"

        target_status = str(target.status_message or "").lower()
        if "fallback" in target_status:
            return "target_state_fallback"
        if "waiting for external" in target_status:
            return "target_state_waiting"
        if not target.actor_id:
            return "target_actor_unbound"

        return ""

    def _build_safe_mode_command(self, command, reason, limits_root):
        safe_limits = self._merge_command_limit_cfg(limits_root, "safe_mode")
        safe_pitch = float(safe_limits.get("max_pitch_cmd", 0.05))
        safe_throttle = float(
            safe_limits.get(
                "moderate_throttle_cmd",
                0.72,
            )
        )

        ego = self._ego_state
        if ego.valid and not ego.on_ground:
            if ego.altitude_world_m < 200.0:
                pitch_cmd = min(safe_pitch, 0.04)
            else:
                pitch_cmd = _clamp((-ego.pitch_deg) * 0.01, -safe_pitch, safe_pitch)
        else:
            pitch_cmd = 0.0

        command.aileron = 0.0
        command.rudder = 0.0
        command.elevator = pitch_cmd
        command.throttle = safe_throttle
        command.command_profile = "safe_mode"
        command.safety_mode_reason = reason

        self._log(
            "WARNING",
            (
                f"safety_mode_active reason={reason} "
                f"source={command.source_node or 'UNKNOWN'} "
                f"branch={command.source_branch or 'unspecified'} "
                f"cmd=({command.throttle:.2f},{command.elevator:.2f},{command.aileron:.2f},{command.rudder:.2f})"
            ),
        )
        return command

    def _advance_mock_backend(self, dt):
        dt = 0.05 if dt <= 0.0 else min(dt, 0.25)
        now = time.monotonic()

        max_speed = float(self.mock_cfg.get("max_speed_mps", 320.0))
        min_speed = float(self.mock_cfg.get("min_speed_mps", 55.0))
        rotate_speed = float(self.mock_cfg.get("rotate_speed_mps", 82.0))
        roll_rate = float(self.mock_cfg.get("rotation_rate_degps", 65.0))
        pitch_rate = float(self.mock_cfg.get("pitch_rate_degps", 30.0))
        target_turn_rate = float(self.mock_cfg.get("target_turn_rate_degps", 1.5))
        target_speed = float(self.mock_cfg.get("target_cruise_speed_mps", 230.0))

        ego = self._ego_state
        enemy = self._enemy_state
        command = self._last_command

        if command.engine_start:
            ego.engine_running = True
        ego.control_enabled = command.control_enabled
        ego.parking_brake = command.parking_brake
        ego.wheel_brake = command.wheel_brake
        ego.gear_down = command.gear_down

        effective_throttle = command.throttle if ego.engine_running and ego.control_enabled else 0.0
        desired_speed = min_speed + (max_speed - min_speed) * effective_throttle

        if ego.on_ground:
            if ego.parking_brake:
                desired_speed = 0.0
            desired_speed *= max(0.0, 1.0 - ego.wheel_brake)

        ego.airspeed_mps += (desired_speed - ego.airspeed_mps) * min(1.0, 0.8 * dt)
        ego.roll_deg = _clamp(ego.roll_deg + command.aileron * roll_rate * dt, -85.0, 85.0)
        ego.pitch_deg = _clamp(ego.pitch_deg + command.elevator * pitch_rate * dt, -20.0, 35.0)

        yaw_rate = (ego.roll_deg * 0.18) + (command.rudder * 10.0)
        ego.yaw_deg = _wrap_angle_deg(ego.yaw_deg + yaw_rate * dt)

        if (
            ego.on_ground
            and not ego.parking_brake
            and ego.airspeed_mps >= rotate_speed
            and command.elevator > 0.1
        ):
            ego.on_ground = False

        horizontal_speed = max(0.0, ego.airspeed_mps * math.cos(math.radians(ego.pitch_deg)))
        vertical_speed = 0.0 if ego.on_ground else ego.airspeed_mps * math.sin(math.radians(ego.pitch_deg))
        ego.vertical_speed_mps = vertical_speed

        ego.position_m.x += math.cos(math.radians(ego.yaw_deg)) * horizontal_speed * dt
        ego.position_m.y += math.sin(math.radians(ego.yaw_deg)) * horizontal_speed * dt

        if ego.on_ground:
            ego.position_m.z = 0.0
            ego.altitude_world_m = 0.0
            ego.altitude_m = 0.0
            ego.vertical_speed_mps = 0.0
        else:
            ego.position_m.z = max(0.0, ego.position_m.z + vertical_speed * dt)
            ego.altitude_world_m = ego.position_m.z
            ego.altitude_m = ego.altitude_world_m
            if ego.position_m.z <= 0.1 and ego.pitch_deg <= 0.0:
                ego.position_m.z = 0.0
                ego.altitude_world_m = 0.0
                ego.altitude_m = 0.0
                ego.on_ground = True

        ego.velocity_mps = Vector3(
            x=math.cos(math.radians(ego.yaw_deg)) * horizontal_speed,
            y=math.sin(math.radians(ego.yaw_deg)) * horizontal_speed,
            z=ego.vertical_speed_mps,
        )
        ego.angle_of_attack_deg = _clamp(command.elevator * 12.0, -8.0, 18.0)
        ego.timestamp_s = now
        ego.state_age_s = 0.0
        ego.actor_id = ego.actor_id or self.ego_actor_id or "mock_ego"
        ego.valid = True
        ego.status_message = "mock_ego_state"

        enemy.airspeed_mps += (target_speed - enemy.airspeed_mps) * min(1.0, 0.5 * dt)
        enemy.yaw_deg = _wrap_angle_deg(enemy.yaw_deg + target_turn_rate * dt)
        enemy.position_m.x += math.cos(math.radians(enemy.yaw_deg)) * enemy.airspeed_mps * dt
        enemy.position_m.y += math.sin(math.radians(enemy.yaw_deg)) * enemy.airspeed_mps * dt
        enemy.position_m.z = max(
            enemy.position_m.z,
            float(self.cfg.get("enemy_initial_state", {}).get("position_m", {}).get("z", 2000.0)),
        )
        enemy.altitude_world_m = enemy.position_m.z
        enemy.altitude_m = enemy.altitude_world_m
        enemy.on_ground = False
        enemy.engine_running = True
        enemy.parking_brake = False
        enemy.wheel_brake = 0.0
        enemy.control_enabled = True
        enemy.actor_id = enemy.actor_id or self.enemy_actor_id or "mock_enemy"
        enemy.velocity_mps = Vector3(
            x=math.cos(math.radians(enemy.yaw_deg)) * enemy.airspeed_mps,
            y=math.sin(math.radians(enemy.yaw_deg)) * enemy.airspeed_mps,
            z=0.0,
        )
        enemy.timestamp_s = now
        enemy.state_age_s = 0.0
        enemy.valid = True
        enemy.status_message = "mock_enemy_state"

    def _ensure_external_adapter_ready(self):
        missing = []
        if self._fetch_state_hook is None:
            missing.append("fetch_state_hook")
        if self._command_hook is None:
            missing.append("command_hook")
        if self.require_actor_ids_in_external and not self.ego_actor_id:
            missing.append("ego_actor_id")
        if self.require_actor_ids_in_external and not self.enemy_actor_id:
            missing.append("enemy_actor_id")

        if not missing:
            return

        message = (
            "External F16 backend is not fully bound. Missing: "
            + ", ".join(missing)
            + ". BT may run, but the hostile aircraft cannot receive real commands."
        )
        self._ego_state = self._invalidate_state(self._ego_state, message)
        self._enemy_state = self._invalidate_state(self._enemy_state, message)
        self._handle_external_error(message)

    def _refresh_external_state(self):
        ego_actor_id = self._require_actor_id("ego")
        enemy_actor_id = self._require_actor_id("enemy")

        ego_payload = self._call_fetch_state_hook(ego_actor_id, "ego")
        enemy_payload = self._call_fetch_state_hook(enemy_actor_id, "enemy")

        self._ego_state = self._coerce_state(
            ego_payload,
            entity_name="ego",
            expected_actor_id=ego_actor_id,
            default_on_ground=self._ego_state.on_ground,
            allow_incomplete=False,
        )
        self._enemy_state = self._coerce_state(
            enemy_payload,
            entity_name="enemy",
            expected_actor_id=enemy_actor_id,
            default_on_ground=self._enemy_state.on_ground,
            allow_incomplete=False,
        )

    def _push_external_command(self, command):
        if self._command_hook is None:
            self._handle_external_error(
                "External command push attempted without a command_hook installed."
            )
            return

        actor_id = self._require_actor_id("ego")
        payload = self._build_command_payload(actor_id, command)
        self._log(
            "INFO",
            (
                f"external command_hook call actor_id={actor_id} seq={payload['command_seq']} "
                f"backend={self._adapter_name or 'direct_hooks'}"
            ),
        )
        response = self._call_command_hook(actor_id, payload)
        self._validate_command_response(actor_id, payload, response)

    def _build_command_payload(self, actor_id, command):
        return {
            "op": "apply_command",
            "actor_id": actor_id,
            "controlled_entity": "ego",
            "agent_id": getattr(self.agent, "agent_id", "no_id_agent"),
            "source_node": command.source_node,
            "source_branch": command.source_branch,
            "command_profile": command.command_profile,
            "safety_mode_reason": command.safety_mode_reason,
            "command_seq": self._command_sequence,
            "timestamp_s": time.monotonic(),
            "throttle": command.throttle,
            "elevator": command.elevator,
            "aileron": command.aileron,
            "rudder": command.rudder,
            "roll_cmd": command.aileron,
            "pitch_cmd": command.elevator,
            "yaw_cmd": command.rudder,
            "receiver_csv4": (
                f"{command.aileron},{command.elevator},{command.rudder},{command.throttle}"
            ),
            "wheel_brake": command.wheel_brake,
            "parking_brake": command.parking_brake,
            "engine_start": command.engine_start,
            "gear_down": command.gear_down,
            "control_enabled": command.control_enabled,
            "controls": {
                "throttle": command.throttle,
                "elevator": command.elevator,
                "aileron": command.aileron,
                "rudder": command.rudder,
            },
            "ground_ops": {
                "wheel_brake": command.wheel_brake,
                "parking_brake": command.parking_brake,
                "engine_start": command.engine_start,
                "gear_down": command.gear_down,
                "control_enabled": command.control_enabled,
            },
        }

    def _call_fetch_state_hook(self, actor_id, entity_name):
        self._log(
            "INFO",
            f"external fetch_state_hook call entity={entity_name} actor_id={actor_id}",
        )
        try:
            return self._fetch_state_hook(actor_id)
        except TypeError:
            return self._fetch_state_hook(actor_id=actor_id, entity_name=entity_name)

    def _call_command_hook(self, actor_id, payload):
        try:
            return self._command_hook(actor_id, payload)
        except TypeError:
            return self._command_hook(actor_id=actor_id, command_dict=payload)

    def _validate_command_response(self, actor_id, payload, response):
        if response is False:
            self._handle_external_error(
                f"command_hook rejected command for actor_id={actor_id} seq={payload['command_seq']}."
            )
            return

        if isinstance(response, dict):
            accepted = response.get("accepted", True)
            ack_actor_id = str(response.get("actor_id", actor_id) or actor_id)
            self._log(
                "INFO",
                f"external command_hook ack actor_id={ack_actor_id} seq={payload['command_seq']} accepted={accepted}",
            )
            if ack_actor_id != actor_id:
                self._handle_external_error(
                    f"command_hook ack actor_id mismatch: expected={actor_id}, got={ack_actor_id}."
                )
            if not accepted:
                self._handle_external_error(
                    f"command_hook reported accepted=False for actor_id={actor_id} seq={payload['command_seq']}."
                )
            return

        if response is None and self.require_command_ack:
            self._handle_external_error(
                (
                    f"command_hook returned no acknowledgment for actor_id={actor_id} "
                    f"seq={payload['command_seq']}. Return {{'accepted': True, 'actor_id': actor_id}}."
                )
            )

    def _copy_state(self, state):
        return AircraftState(
            actor_id=state.actor_id,
            position_m=Vector3(state.position_m.x, state.position_m.y, state.position_m.z),
            velocity_mps=Vector3(state.velocity_mps.x, state.velocity_mps.y, state.velocity_mps.z),
            ground_speed_mps=state.ground_speed_mps,
            yaw_deg=state.yaw_deg,
            pitch_deg=state.pitch_deg,
            roll_deg=state.roll_deg,
            airspeed_mps=state.airspeed_mps,
            altitude_m=state.altitude_m,
            altitude_world_m=state.altitude_world_m,
            altitude_agl_m=state.altitude_agl_m,
            vertical_speed_mps=state.vertical_speed_mps,
            angle_of_attack_deg=state.angle_of_attack_deg,
            timestamp_s=state.timestamp_s,
            state_age_s=state.state_age_s,
            on_ground=state.on_ground,
            engine_running=state.engine_running,
            parking_brake=state.parking_brake,
            wheel_brake=state.wheel_brake,
            gear_down=state.gear_down,
            control_enabled=state.control_enabled,
            valid=state.valid,
            status_message=state.status_message,
            state_source=state.state_source,
        )

    def _invalidate_state(self, previous_state, reason):
        state = self._copy_state(previous_state)
        state.valid = False
        state.status_message = reason
        state.timestamp_s = time.monotonic()
        return state

    def _handle_external_error(self, message):
        self._log("ERROR", message)
        if self.strict_external_adapter or self.strict_state_validation:
            raise RuntimeError(message)

    def _coerce_state(
        self,
        payload,
        entity_name,
        expected_actor_id="",
        default_on_ground=False,
        allow_incomplete=False,
    ):
        base_state = (
            getattr(self, "_ego_state", AircraftState())
            if entity_name == "ego"
            else getattr(self, "_enemy_state", AircraftState())
        )

        if payload is None:
            return self._invalidate_state(
                base_state,
                f"Missing {entity_name} state payload from external adapter.",
            )

        if isinstance(payload, AircraftState):
            state = self._copy_state(payload)
            state.actor_id = state.actor_id or expected_actor_id or entity_name
            if state.altitude_world_m == 0.0 and state.altitude_m != 0.0:
                state.altitude_world_m = state.altitude_m
            state.altitude_agl_m = getattr(state, "altitude_agl_m", 0.0)
            if state.altitude_m == 0.0 and state.altitude_world_m != 0.0:
                state.altitude_m = state.altitude_world_m
            if state.altitude_world_m == 0.0 and state.altitude_m == 0.0:
                state.altitude_world_m = state.position_m.z
                state.altitude_m = state.position_m.z
            return self._validate_state(state, entity_name, expected_actor_id)

        if not isinstance(payload, dict):
            return self._invalidate_state(
                base_state,
                (
                    f"{entity_name} state payload must be a dict or AircraftState, "
                    f"got {type(payload).__name__}."
                ),
            )

        required_fields = [
            ("position_m", payload.get("position_m")),
            ("velocity_mps", payload.get("velocity_mps")),
            ("yaw_deg", payload.get("yaw_deg")),
            ("pitch_deg", payload.get("pitch_deg")),
            ("roll_deg", payload.get("roll_deg")),
            ("airspeed_mps", payload.get("airspeed_mps")),
            ("on_ground", payload.get("on_ground")),
        ]
        if not allow_incomplete:
            missing = [name for name, value in required_fields if value is None]
            if missing:
                return self._invalidate_state(
                    base_state,
                    f"{entity_name} state missing required fields: {', '.join(missing)}.",
                )

        position = payload.get("position_m", {}) or {}
        velocity = payload.get("velocity_mps", {}) or {}
        if not allow_incomplete:
            nested_missing = []
            for axis in ("x", "y", "z"):
                if position.get(axis) is None:
                    nested_missing.append(f"position_m.{axis}")
                if velocity.get(axis) is None:
                    nested_missing.append(f"velocity_mps.{axis}")
            if nested_missing:
                return self._invalidate_state(
                    base_state,
                    f"{entity_name} state missing required fields: {', '.join(nested_missing)}.",
                )

        altitude_world_m = payload.get(
            "altitude_world_m", payload.get("altitude_m", position.get("z", 0.0))
        )
        altitude_world_m = _safe_float(altitude_world_m, position.get("z", 0.0))
        position_z = _safe_float(position.get("z", altitude_world_m), altitude_world_m)

        state = AircraftState(
            actor_id=str(payload.get("actor_id", expected_actor_id or entity_name) or "").strip(),
            position_m=Vector3(
                x=_safe_float(position.get("x", 0.0)),
                y=_safe_float(position.get("y", 0.0)),
                z=position_z,
            ),
            velocity_mps=Vector3(
                x=_safe_float(velocity.get("x", 0.0)),
                y=_safe_float(velocity.get("y", 0.0)),
                z=_safe_float(velocity.get("z", 0.0)),
            ),
            yaw_deg=_safe_float(payload.get("yaw_deg", 0.0)),
            pitch_deg=_safe_float(payload.get("pitch_deg", 0.0)),
            roll_deg=_safe_float(payload.get("roll_deg", 0.0)),
            airspeed_mps=_safe_float(payload.get("airspeed_mps", 0.0)),
            ground_speed_mps=_safe_float(
                payload.get("ground_speed_mps", payload.get("airspeed_mps", 0.0)),
                0.0,
            ),
            altitude_m=altitude_world_m,
            altitude_world_m=altitude_world_m,
            altitude_agl_m=_safe_float(payload.get("altitude_agl_m", 0.0), 0.0),
            vertical_speed_mps=_safe_float(payload.get("vertical_speed_mps", 0.0)),
            angle_of_attack_deg=_safe_float(payload.get("angle_of_attack_deg", 0.0)),
            timestamp_s=time.monotonic(),
            state_age_s=max(
                0.0,
                _safe_float(payload.get("state_age_s", payload.get("data_age_s", 0.0)), 0.0),
            ),
            on_ground=bool(payload.get("on_ground", default_on_ground)),
            engine_running=bool(payload.get("engine_running", False)),
            parking_brake=bool(payload.get("parking_brake", False)),
            wheel_brake=_clamp(_safe_float(payload.get("wheel_brake", 0.0)), 0.0, 1.0),
            gear_down=bool(payload.get("gear_down", True)),
            control_enabled=bool(payload.get("control_enabled", True)),
            valid=bool(payload.get("valid", True)),
            status_message=str(payload.get("status_message", "")),
            state_source=str(payload.get("state_source", "")),
        )
        return self._validate_state(state, entity_name, expected_actor_id)

    def _validate_state(self, state, entity_name, expected_actor_id):
        numeric_fields = [
            state.position_m.x,
            state.position_m.y,
            state.position_m.z,
            state.velocity_mps.x,
            state.velocity_mps.y,
            state.velocity_mps.z,
            state.yaw_deg,
            state.pitch_deg,
            state.roll_deg,
            state.airspeed_mps,
            state.altitude_m,
            state.altitude_world_m,
            state.altitude_agl_m,
            state.vertical_speed_mps,
            state.angle_of_attack_deg,
            state.state_age_s,
            state.wheel_brake,
            state.ground_speed_mps,
        ]
        if not all(_is_finite_number(value) for value in numeric_fields):
            state.valid = False
            state.status_message = f"{entity_name} state contains non-finite numeric values."
            self._handle_invalid_state(state)
            return state

        if expected_actor_id and state.actor_id and state.actor_id != expected_actor_id:
            state.valid = False
            state.status_message = (
                f"{entity_name} actor_id mismatch: expected={expected_actor_id}, got={state.actor_id}."
            )
            self._handle_invalid_state(state)
            return state

        altitude_delta = abs(state.altitude_world_m - state.position_m.z)
        if altitude_delta > self.altitude_consistency_tolerance_m:
            state.valid = False
            state.status_message = (
                f"{entity_name} altitude mismatch: altitude_world_m={state.altitude_world_m:.1f}, "
                f"position_m.z={state.position_m.z:.1f}."
            )
            self._handle_invalid_state(state)
            return state

        if self.max_state_age_s >= 0.0 and state.state_age_s > self.max_state_age_s:
            state.valid = False
            state.status_message = (
                f"{entity_name} state is stale: age={state.state_age_s:.3f}s exceeds "
                f"{self.max_state_age_s:.3f}s."
            )
            self._handle_invalid_state(state)
            return state

        if state.airspeed_mps < 0.0:
            state.valid = False
            state.status_message = f"{entity_name} airspeed_mps cannot be negative."
            self._handle_invalid_state(state)
            return state

        state.altitude_m = state.altitude_world_m
        state.status_message = state.status_message or f"{entity_name}_state_ok"
        return state

    def _handle_invalid_state(self, state):
        self._log("ERROR", state.status_message)
        if self.strict_state_validation:
            raise RuntimeError(state.status_message)

    def _maybe_log_state_snapshots(self):
        if not self.state_logging:
            return

        now = time.monotonic()
        if now - self._last_state_log_s < self.state_log_interval_s:
            return

        self._last_state_log_s = now
        self._log("INFO", self._format_state_line("ego", self._ego_state))
        self._log("INFO", self._format_state_line("enemy", self._enemy_state))

    def _format_state_line(self, entity_name, state):
        return (
            f"state entity={entity_name} actor_id={state.actor_id or 'UNBOUND'} "
            f"pos=({state.position_m.x:.1f},{state.position_m.y:.1f},{state.position_m.z:.1f}) "
            f"vel=({state.velocity_mps.x:.1f},{state.velocity_mps.y:.1f},{state.velocity_mps.z:.1f}) "
            f"att=({state.yaw_deg:.1f},{state.pitch_deg:.1f},{state.roll_deg:.1f}) "
            f"spd={state.airspeed_mps:.1f}mps gs={state.ground_speed_mps:.1f}mps on_ground={state.on_ground} "
            f"alt={state.altitude_world_m:.1f}m agl={state.altitude_agl_m:.1f}m engine_running={state.engine_running} "
            f"parking_brake={state.parking_brake} wheel_brake={state.wheel_brake:.2f} "
            f"control_enabled={state.control_enabled} valid={state.valid} "
            f"source={state.state_source or 'unknown'} status={state.status_message}"
        )

    def _maybe_warn_ground_motion_blockers(self):
        throttle_threshold = float(
            self.cfg.get("ground_ops", {}).get("movement_check_throttle_threshold", 0.35)
        )
        min_rolling_speed_mps = float(
            self.cfg.get("ground_ops", {}).get("movement_check_speed_threshold_mps", 3.0)
        )

        ego = self._ego_state
        if getattr(ego, "status_message", "") == "udp_initial_fallback":
            return
        if not ego.on_ground or self._last_command.throttle < throttle_threshold:
            return
        if ego.airspeed_mps >= min_rolling_speed_mps:
            return

        now = time.monotonic()
        if now - self._last_motion_warning_s < self.movement_check_interval_s:
            return

        self._last_motion_warning_s = now
        blockers = []
        if not ego.engine_running:
            blockers.append("engine_running=False")
        if ego.parking_brake:
            blockers.append("parking_brake=True")
        if ego.wheel_brake > 0.05:
            blockers.append(f"wheel_brake={ego.wheel_brake:.2f}")
        if not ego.control_enabled:
            blockers.append("control_enabled=False")
        if self.backend_mode == "external" and self._command_hook is None:
            blockers.append("command_hook_missing")
        if self.backend_mode == "external" and not self.ego_actor_id:
            blockers.append("ego_actor_id_missing")
        if not blockers:
            blockers.append("command path not applying inputs or actor_id mismatch")

        self._log(
            "ERROR",
            (
                f"ego actor={self.get_command_target_id()} is not rolling on the ground "
                f"despite throttle={self._last_command.throttle:.2f}. blockers={', '.join(blockers)}"
            ),
        )

    def _require_actor_id(self, entity_name):
        actor_id = self.ego_actor_id if entity_name == "ego" else self.enemy_actor_id
        actor_id = str(actor_id or "").strip()
        if actor_id:
            return actor_id

        if self.backend_mode != "external" or not self.require_actor_ids_in_external:
            return entity_name

        self._handle_external_error(
            (
                f"External {entity_name} actor binding is empty. "
                f"Set f16_interface.{entity_name}_actor_id to the real Unreal pawn / actor id."
            )
        )
        return entity_name

    def _log(self, level, message):
        getattr(LOGGER, level.lower(), LOGGER.info)(message)
        if self.debug_logging or level in ("WARNING", "ERROR"):
            print(f"[F16IF:{level}] {message}")
