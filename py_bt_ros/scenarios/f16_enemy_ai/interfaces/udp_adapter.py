import copy
import json
import math
import socket


def _safe_float(value, default=0.0):
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return float(default)
    if not math.isfinite(numeric):
        return float(default)
    return numeric


def _safe_bool(value, default=False):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return bool(default)


class F16UdpBridgeAdapter:
    """
    UDP bridge adapter for the current Unreal hostile F-16 path.

    Command path:
      csv4 in the receiver order roll,pitch,yaw,throttle

    Internal control mapping:
      aileron -> roll
      elevator -> pitch
      rudder -> yaw
      throttle -> throttle

    State path:
      JSON datagrams from Unreal on the configured UDP state port
    """

    def __init__(self, config, interface=None, agent=None):
        self.config = config or {}
        self.interface = interface
        self.agent = agent

        interface_cfg = self.config.get("f16_interface", {})
        udp_cfg = interface_cfg.get("udp_adapter", {})

        self.host = str(udp_cfg.get("host", "127.0.0.1"))
        self.command_port = int(udp_cfg.get("command_port", 5005))
        self.command_bind_host = str(udp_cfg.get("command_bind_host", "0.0.0.0"))
        self.command_bind_port = int(udp_cfg.get("command_bind_port", 0))
        self.state_host = str(udp_cfg.get("state_host", "0.0.0.0"))
        self.state_port = int(udp_cfg.get("state_port", 5007))
        self.encoding = str(udp_cfg.get("encoding", "utf-8"))
        self.state_socket_timeout_s = float(udp_cfg.get("state_socket_timeout_s", 0.001))
        self.enable_state_listener = bool(udp_cfg.get("enable_state_listener", True))
        self.allow_initial_state_fallback = bool(
            udp_cfg.get("allow_initial_state_fallback", True)
        )
        self.require_states_array = bool(udp_cfg.get("require_states_array", True))
        self.expected_state_source = str(
            udp_cfg.get("expected_state_source", "PyAircraftUdpStateSenderSubsystem")
        ).strip()
        self.debug_logging = bool(udp_cfg.get("debug_logging", True))
        self.command_limits = interface_cfg.get("command_limits", {})

        self.ego_actor_id = str(interface_cfg.get("ego_actor_id", "") or "").strip()
        self.enemy_actor_id = str(interface_cfg.get("enemy_actor_id", "") or "").strip()
        self.single_state_actor_id = str(
            udp_cfg.get("single_state_actor_id", "") or self.ego_actor_id or ""
        ).strip()

        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._command_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._command_socket.bind((self.command_bind_host, self.command_bind_port))
        self._state_socket = None
        self._state_cache = {}
        self._warned_missing_state = set()
        self._announced_live_state_actors = set()

        command_local_host, command_local_port = self._safe_getsockname(self._command_socket)
        print(
            "[F16UdpBridgeAdapter] "
            f"command socket bound local={command_local_host}:{command_local_port} "
            f"send_target={self.host}:{self.command_port}"
        )

        self._ensure_state_socket()

    def close(self):
        if self._state_socket is not None:
            self._state_socket.close()
            self._state_socket = None
        if self._command_socket is not None:
            self._command_socket.close()
            self._command_socket = None

    def fetch_state(self, actor_id):
        actor_id = str(actor_id or "").strip()
        self._drain_state_socket()

        cached = self._state_cache.get(actor_id)
        if cached is not None:
            self._announce_live_state_use(actor_id, cached)
            return copy.deepcopy(cached)

        fallback = self._build_fallback_state(actor_id)
        if fallback is not None:
            if actor_id not in self._warned_missing_state:
                available_live = sorted(self._state_cache.keys())
                print(
                    "[F16UdpBridgeAdapter] "
                    f"using initial fallback state for actor_id={actor_id or 'UNBOUND'} "
                    f"available_live_actor_ids={available_live}"
                )
                self._warned_missing_state.add(actor_id)
            return fallback

        raise RuntimeError(
            f"UDP state unavailable for actor_id={actor_id or 'UNBOUND'} on port {self.state_port}."
        )

    def send_command(self, actor_id, command_dict):
        roll = float(command_dict.get("roll_cmd", command_dict.get("aileron", 0.0)))
        pitch = float(command_dict.get("pitch_cmd", command_dict.get("elevator", 0.0)))
        yaw = float(command_dict.get("yaw_cmd", command_dict.get("rudder", 0.0)))
        throttle = float(command_dict.get("throttle", 0.0))
        source_node = str(command_dict.get("source_node", "") or "").strip() or "UNKNOWN"
        source_branch = str(command_dict.get("source_branch", "") or "").strip() or "unspecified"
        profile_name = str(command_dict.get("command_profile", "") or "").strip() or "default"

        raw_values = (roll, pitch, yaw, throttle)
        roll, pitch, yaw, throttle = self._apply_final_output_limits(
            roll,
            pitch,
            yaw,
            throttle,
            profile_name,
        )

        payload = f"{roll},{pitch},{yaw},{throttle}"
        encoded = payload.encode(self.encoding)
        sent = self._command_socket.sendto(encoded, (self.host, self.command_port))
        local_host, local_port = self._safe_getsockname(self._command_socket)

        print(
            "[F16UdpBridgeAdapter] "
            f"source={source_node} branch={source_branch} "
            f"profile={profile_name} "
            f"actor_id={actor_id or self.ego_actor_id or 'UNBOUND'} "
            f"local={local_host}:{local_port} "
            f"sendto={self.host}:{self.command_port} "
            f"csv4(roll,pitch,yaw,throttle)={payload} "
            f"raw=({raw_values[0]:.2f},{raw_values[1]:.2f},{raw_values[2]:.2f},{raw_values[3]:.2f}) "
            f"bytes_sent={sent}"
        )

        return {
            "accepted": True,
            "actor_id": str(actor_id or self.ego_actor_id or ""),
            "payload": payload,
            "receiver_axes": {
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "throttle": throttle,
            },
        }

    def _apply_final_output_limits(self, roll, pitch, yaw, throttle, profile_name):
        merged = dict(self.command_limits.get("default", {}) or {})
        merged.update(self.command_limits.get(profile_name, {}) or {})

        max_roll = float(merged.get("max_roll_cmd", 0.15))
        max_pitch = float(merged.get("max_pitch_cmd", 0.10))
        max_yaw = float(merged.get("max_yaw_cmd", 0.10))
        min_throttle = float(merged.get("min_throttle_cmd", 0.60))
        max_throttle = float(merged.get("max_throttle_cmd", 0.85))

        clamped_roll = max(-max_roll, min(max_roll, float(roll)))
        clamped_pitch = max(-max_pitch, min(max_pitch, float(pitch)))
        clamped_yaw = max(-max_yaw, min(max_yaw, float(yaw)))
        clamped_throttle = max(min_throttle, min(max_throttle, float(throttle)))

        if self.debug_logging and (
            abs(clamped_roll - roll) > 1e-6
            or abs(clamped_pitch - pitch) > 1e-6
            or abs(clamped_yaw - yaw) > 1e-6
            or abs(clamped_throttle - throttle) > 1e-6
        ):
            print(
                "[F16UdpBridgeAdapter] "
                f"final_output_limited profile={profile_name} "
                f"raw=({roll:.2f},{pitch:.2f},{yaw:.2f},{throttle:.2f}) "
                f"clamped=({clamped_roll:.2f},{clamped_pitch:.2f},{clamped_yaw:.2f},{clamped_throttle:.2f})"
            )

        return clamped_roll, clamped_pitch, clamped_yaw, clamped_throttle

    def _ensure_state_socket(self):
        if not self.enable_state_listener or self._state_socket is not None:
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.state_host, self.state_port))
        sock.settimeout(self.state_socket_timeout_s)
        self._state_socket = sock

        print(
            "[F16UdpBridgeAdapter] "
            f"state listener bound local={self.state_host}:{self.state_port}"
        )

    def _safe_getsockname(self, sock):
        try:
            address = sock.getsockname()
        except OSError:
            return "UNKNOWN", -1

        if not isinstance(address, tuple) or len(address) < 2:
            return "UNKNOWN", -1

        return str(address[0]), int(address[1])

    def _drain_state_socket(self):
        if self._state_socket is None:
            return

        while True:
            try:
                data, addr = self._state_socket.recvfrom(65535)
            except socket.timeout:
                break
            except BlockingIOError:
                break
            except OSError:
                break

            self._handle_state_packet(data, addr)

    def _handle_state_packet(self, data, addr):
        try:
            text = data.decode(self.encoding).strip()
        except UnicodeDecodeError:
            if self.debug_logging:
                print(
                    "[F16UdpBridgeAdapter] "
                    f"ignored non-text UDP state packet from {addr} bytes={len(data)}"
                )
            return

        if not text:
            return

        if self.debug_logging:
            print(f"[F16UdpBridgeAdapter] raw_state from={addr} text={text}")

        try:
            packet = json.loads(text)
        except json.JSONDecodeError:
            if self.debug_logging:
                print(
                    "[F16UdpBridgeAdapter] "
                    f"ignored non-JSON UDP state packet from {addr}: {text}"
                )
            return

        if isinstance(packet, dict):
            has_states_array = isinstance(packet.get("states"), list)
            packet_source = str(packet.get("source", "") or "").strip()

            if self.require_states_array and not has_states_array:
                if self.debug_logging:
                    print(
                        "[F16UdpBridgeAdapter] "
                        "ignored UDP state packet without states array "
                        f"from {addr}: keys={sorted(packet.keys())}"
                    )
                return

            if self.expected_state_source and has_states_array and packet_source != self.expected_state_source:
                if self.debug_logging:
                    print(
                        "[F16UdpBridgeAdapter] "
                        f"ignored UDP state packet with unexpected source='{packet_source}' "
                        f"expected='{self.expected_state_source}'"
                    )
                return

        packet_actor_ids = []
        for record in self._iter_state_records(packet):
            state = self._normalise_state_record(record)
            actor_id = str(state.get("actor_id", "") or "").strip()
            if actor_id:
                self._state_cache[actor_id] = state
                self._warned_missing_state.discard(actor_id)
                packet_actor_ids.append(actor_id)

            if self.debug_logging:
                print(
                    "[F16UdpBridgeAdapter] "
                    f"cached LIVE UDP state actor_id={actor_id or 'UNBOUND'} "
                    f"role={str(state.get('role', '') or '').strip().lower() or 'none'} aircraft_name={state.get('aircraft_name', '') or 'unknown'} "
                    f"source={state.get('state_source', 'unknown')} "
                    f"pos={state['position_m']} vel={state['velocity_mps']} "
                    f"airspeed={state['airspeed_mps']:.2f} on_ground={state['on_ground']} "
                    f"yaw={state['yaw_deg']:.2f} pitch={state['pitch_deg']:.2f} roll={state['roll_deg']:.2f}"
                )

        if packet_actor_ids:
            unique_actor_ids = sorted(set(packet_actor_ids))
            print(
                "[F16UdpBridgeAdapter] "
                f"live UDP packet actor_ids={unique_actor_ids}"
            )

    def _iter_state_records(self, packet):
        if isinstance(packet, list):
            for item in packet:
                if isinstance(item, dict):
                    yield item
            return

        if not isinstance(packet, dict):
            return

        for key in ("states", "actors", "aircraft", "vehicles"):
            value = packet.get(key)
            if isinstance(value, list):
                for item in value:
                    if isinstance(item, dict):
                        yield item
            elif isinstance(value, dict):
                for actor_id, item in value.items():
                    if isinstance(item, dict):
                        child = dict(item)
                        child.setdefault("actor_id", str(actor_id))
                        yield child

    def _normalise_state_record(self, record):
        actor_id = self._extract_actor_id(record)
        if not actor_id and self.single_state_actor_id:
            actor_id = self.single_state_actor_id

        position = self._extract_vector(
            record,
            vector_keys=("position_m", "position", "location", "loc"),
            axis_keys=(("x", "y", "z"), ("pos_x", "pos_y", "pos_z"), ("location_x", "location_y", "location_z")),
        )
        velocity = self._extract_vector(
            record,
            vector_keys=("velocity_mps", "velocity", "linear_velocity", "vel"),
            axis_keys=(("vx", "vy", "vz"), ("vel_x", "vel_y", "vel_z"), ("speed_x", "speed_y", "speed_z")),
        )

        yaw = self._first_numeric(record, ("yaw_deg", "yaw", "heading_deg", "heading"), 0.0)
        pitch = self._first_numeric(record, ("pitch_deg", "pitch"), 0.0)
        roll = self._first_numeric(record, ("roll_deg", "roll", "bank_deg", "bank"), 0.0)
        altitude = self._first_numeric(
            record,
            ("altitude_world_m", "altitude_m", "altitude", "alt"),
            position["z"],
        )
        altitude_agl = self._first_numeric(
            record,
            ("altitude_agl_m", "agl_m", "altitude_agl"),
            0.0,
        )
        airspeed = self._first_numeric(
            record,
            ("airspeed_mps", "airspeed", "speed_mps", "speed"),
            math.sqrt(
                (velocity["x"] ** 2) + (velocity["y"] ** 2) + (velocity["z"] ** 2)
            ),
        )
        ground_speed = self._first_numeric(
            record,
            ("ground_speed_mps", "groundspeed_mps", "ground_speed"),
            math.sqrt((velocity["x"] ** 2) + (velocity["y"] ** 2)),
        )
        vertical_speed = self._first_numeric(record, ("vertical_speed_mps", "vz"), velocity["z"])
        aoa = self._first_numeric(record, ("angle_of_attack_deg", "aoa_deg", "aoa"), 0.0)
        state_age = self._first_numeric(record, ("state_age_s", "data_age_s", "age_s"), 0.0)
        on_ground = self._first_bool(
            record,
            ("on_ground", "grounded", "is_grounded"),
            altitude <= 0.1 and airspeed < 1.0,
        )

        return {
            "actor_id": actor_id,
            "aircraft_name": str(record.get("aircraft_name", record.get("actor_name", actor_id))),
            "role": str(record.get("role", "")),
            "position_m": position,
            "velocity_mps": velocity,
            "yaw_deg": yaw,
            "pitch_deg": pitch,
            "roll_deg": roll,
            "airspeed_mps": airspeed,
            "ground_speed_mps": ground_speed,
            "altitude_m": altitude,
            "altitude_world_m": altitude,
            "altitude_agl_m": altitude_agl,
            "vertical_speed_mps": vertical_speed,
            "angle_of_attack_deg": aoa,
            "on_ground": on_ground,
            "engine_running": self._first_bool(record, ("engine_running",), True),
            "parking_brake": self._first_bool(record, ("parking_brake",), False),
            "wheel_brake": self._first_numeric(record, ("wheel_brake",), 0.0),
            "gear_down": self._first_bool(record, ("gear_down",), True),
            "control_enabled": self._first_bool(record, ("control_enabled",), True),
            "state_age_s": state_age,
            "valid": True,
            "status_message": str(record.get("status_message", "udp_live_state")),
            "state_source": str(record.get("state_source", "")),
        }

    def _announce_live_state_use(self, actor_id, state):
        actor_id = str(actor_id or state.get("actor_id", "") or "UNBOUND")
        if actor_id in self._announced_live_state_actors:
            return

        self._announced_live_state_actors.add(actor_id)
        print(
            "[F16UdpBridgeAdapter] "
            f"using LIVE UDP state for actor_id={actor_id} "
            f"status={state.get('status_message', 'udp_live_state')} "
            f"role={state.get('role', '') or 'none'} "
            f"aircraft_name={state.get('aircraft_name', '') or 'unknown'} "
            f"source={state.get('state_source', 'unknown')} "
            f"pos={state.get('position_m')} vel={state.get('velocity_mps')} "
            f"airspeed={_safe_float(state.get('airspeed_mps', 0.0)):.2f} "
            f"on_ground={state.get('on_ground')}"
        )

    def _extract_actor_id(self, record):
        for key in ("actor_id", "id", "name", "pawn", "vehicle_id", "aircraft_id"):
            value = record.get(key)
            if value:
                return str(value).strip()
        return ""

    def _extract_vector(self, record, vector_keys, axis_keys):
        for key in vector_keys:
            value = record.get(key)
            if isinstance(value, dict):
                return {
                    "x": _safe_float(value.get("x", value.get("X", 0.0))),
                    "y": _safe_float(value.get("y", value.get("Y", 0.0))),
                    "z": _safe_float(value.get("z", value.get("Z", 0.0))),
                }
            if isinstance(value, (list, tuple)) and len(value) >= 3:
                return {
                    "x": _safe_float(value[0]),
                    "y": _safe_float(value[1]),
                    "z": _safe_float(value[2]),
                }

        for x_key, y_key, z_key in axis_keys:
            if any(k in record for k in (x_key, y_key, z_key)):
                return {
                    "x": _safe_float(record.get(x_key, 0.0)),
                    "y": _safe_float(record.get(y_key, 0.0)),
                    "z": _safe_float(record.get(z_key, 0.0)),
                }

        return {"x": 0.0, "y": 0.0, "z": 0.0}

    def _first_numeric(self, record, keys, default):
        for key in keys:
            if key in record:
                return _safe_float(record.get(key), default)
        return _safe_float(default, default)

    def _first_bool(self, record, keys, default):
        for key in keys:
            if key in record:
                return _safe_bool(record.get(key), default)
        return _safe_bool(default, default)

    def _build_fallback_state(self, actor_id):
        if not self.allow_initial_state_fallback:
            return None

        interface_cfg = self.config.get("f16_interface", {})
        if actor_id == self.ego_actor_id:
            source = interface_cfg.get("ego_initial_state", {})
        elif actor_id == self.enemy_actor_id:
            source = interface_cfg.get("enemy_initial_state", {})
        else:
            source = {}

        if not source:
            return None

        state = copy.deepcopy(source)
        position = dict(state.get("position_m", {}) or {})
        velocity = dict(state.get("velocity_mps", {}) or {})
        altitude = _safe_float(state.get("altitude_m", position.get("z", 0.0)))

        return {
            "actor_id": actor_id,
            "position_m": {
                "x": _safe_float(position.get("x", 0.0)),
                "y": _safe_float(position.get("y", 0.0)),
                "z": _safe_float(position.get("z", altitude)),
            },
            "velocity_mps": {
                "x": _safe_float(velocity.get("x", 0.0)),
                "y": _safe_float(velocity.get("y", 0.0)),
                "z": _safe_float(velocity.get("z", 0.0)),
            },
            "yaw_deg": _safe_float(state.get("yaw_deg", 0.0)),
            "pitch_deg": _safe_float(state.get("pitch_deg", 0.0)),
            "roll_deg": _safe_float(state.get("roll_deg", 0.0)),
            "airspeed_mps": _safe_float(state.get("airspeed_mps", 0.0)),
            "ground_speed_mps": _safe_float(state.get("ground_speed_mps", state.get("airspeed_mps", 0.0))),
            "altitude_m": altitude,
            "altitude_world_m": altitude,
            "altitude_agl_m": _safe_float(state.get("altitude_agl_m", 0.0)),
            "vertical_speed_mps": _safe_float(state.get("vertical_speed_mps", 0.0)),
            "angle_of_attack_deg": _safe_float(state.get("angle_of_attack_deg", 0.0)),
            "on_ground": _safe_bool(state.get("on_ground", False), False),
            "engine_running": _safe_bool(state.get("engine_running", False), False),
            "parking_brake": _safe_bool(state.get("parking_brake", False), False),
            "wheel_brake": _safe_float(state.get("wheel_brake", 0.0)),
            "gear_down": _safe_bool(state.get("gear_down", True), True),
            "control_enabled": _safe_bool(state.get("control_enabled", True), True),
            "state_age_s": 0.0,
            "valid": True,
            "status_message": "udp_initial_fallback",
            "state_source": "udp_initial_fallback",
        }
