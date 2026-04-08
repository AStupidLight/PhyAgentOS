"""Unitree G1 real-hardware driver backed by unitree_sdk2_python."""

from __future__ import annotations

import math
from datetime import datetime
from pathlib import Path
import sys
import time
from typing import Any

from hal.base_driver import BaseDriver

_PROFILES_DIR = Path(__file__).resolve().parent.parent / "profiles"
_CHANNEL_INIT_CACHE: set[str] = set()


def _default_unitree_sdk_path() -> str:
    candidate = Path(__file__).resolve().parents[2] / "unitree_sdk2_python"
    return str(candidate) if candidate.exists() else ""


class G1Driver(BaseDriver):
    """Minimal real-hardware G1 locomotion driver."""

    ROBOT_ID = "g1_001"
    _G1_HIGH_STAND_VALUE = float((1 << 32) - 1)
    _G1_LOW_STAND_VALUE = 0.0

    def __init__(self, gui: bool = False, **kwargs: Any):
        del gui
        self._objects: dict[str, dict] = {}
        self._unitree_sdk_path = str(kwargs.get("unitree_sdk_path") or _default_unitree_sdk_path())
        self._network_interface = str(kwargs.get("network_interface") or "")
        self._stand_up_on_connect = bool(kwargs.get("stand_up_on_connect", False))
        self._linear_speed_mps = max(0.05, float(kwargs.get("linear_speed_mps", 0.25)))
        self._angular_speed_rps = max(0.1, float(kwargs.get("angular_speed_rps", 0.8)))
        self._command_dt_s = max(0.05, float(kwargs.get("command_dt_s", 0.2)))
        self._motion_timeout_s = max(1.0, float(kwargs.get("motion_timeout_s", 10.0)))
        self._reconnect_policy = str(kwargs.get("reconnect_policy", "manual"))

        self._channel_factory_initialize = None
        self._loco_client = None
        self._rpc_ok = 0
        self._shake_hand_stage_state: int | bool = -1
        self._last_error: str | None = None
        self._runtime_state = {"robots": {self.ROBOT_ID: self._make_robot_state()}}

    def get_profile_path(self) -> Path:
        return _PROFILES_DIR / "g1.md"

    def load_scene(self, scene: dict[str, dict]) -> None:
        self._objects = dict(scene)

    def connect(self) -> bool:
        state = self._robot_state(self.ROBOT_ID)
        conn = dict(state["connection_state"])
        try:
            self._ensure_loco_client()
            if self._stand_up_on_connect:
                self._sdk_damp()
                time.sleep(0.5)
                self._sdk_squat_to_stand()
            conn.update(
                {
                    "status": "connected",
                    "transport": "dds",
                    "host": self._network_interface or "default",
                    "port": 0,
                    "last_heartbeat": self._stamp(),
                    "last_error": None,
                }
            )
            self._last_error = None
            self._update_motion_state(status="idle", mode="idle", last_error=None)
            state["connection_state"] = conn
            return True
        except Exception as exc:
            self._loco_client = None
            self._last_error = str(exc)
            conn.update(
                {
                    "status": "disconnected",
                    "transport": "dds",
                    "host": self._network_interface or "default",
                    "port": 0,
                    "last_heartbeat": self._stamp(),
                    "last_error": self._last_error,
                }
            )
            state["connection_state"] = conn
            self._update_motion_state(status="failed", mode="idle", last_error=self._last_error)
            return False

    def disconnect(self) -> None:
        if self._loco_client is not None:
            try:
                self._sdk_stop_move()
            except Exception:
                pass
        self._loco_client = None
        state = self._robot_state(self.ROBOT_ID)
        conn = dict(state["connection_state"])
        conn.update(
            {
                "status": "disconnected",
                "last_heartbeat": self._stamp(),
                "last_error": None,
            }
        )
        state["connection_state"] = conn
        self._update_motion_state(status="idle", mode="idle", last_error=None)

    def is_connected(self) -> bool:
        return self._loco_client is not None and self._robot_state(self.ROBOT_ID)["connection_state"].get("status") == "connected"

    def health_check(self) -> bool:
        state = self._robot_state(self.ROBOT_ID)
        conn = dict(state["connection_state"])
        if self._loco_client is not None:
            try:
                self._safe_rpc("GetServerApiVersion")
            except RuntimeError as exc:
                self._last_error = str(exc)
                self._loco_client = None
                conn["status"] = "disconnected"
                conn["last_heartbeat"] = self._stamp()
                conn["last_error"] = self._last_error
                state["connection_state"] = conn
                return False
            conn["status"] = "connected"
            conn["last_heartbeat"] = self._stamp()
            conn["last_error"] = None
            state["connection_state"] = conn
            return True

        if self._reconnect_policy == "auto":
            conn["status"] = "reconnecting"
            conn["reconnect_attempts"] = conn.get("reconnect_attempts", 0) + 1
            conn["last_error"] = self._last_error or "connection_lost"
            state["connection_state"] = conn
            return self.connect()

        conn["status"] = "disconnected"
        conn["last_heartbeat"] = self._stamp()
        conn["last_error"] = self._last_error
        state["connection_state"] = conn
        return False

    def execute_action(self, action_type: str, params: dict) -> str:
        if action_type == "connect_robot":
            return "Robot connection established." if self.connect() else f"Connection error: {self._last_error or 'failed to connect'}"
        if action_type == "disconnect_robot":
            self.disconnect()
            return "Robot connection closed."
        if action_type == "reconnect_robot":
            self.disconnect()
            return "Robot reconnected." if self.connect() else f"Connection error: {self._last_error or 'failed to reconnect'}"
        if action_type == "check_connection":
            return "connected" if self.health_check() else "disconnected"

        if not self.is_connected() and not self.connect():
            self._update_motion_state(status="failed", mode="idle", last_error="disconnected")
            return "Connection error: robot is not connected."

        try:
            if action_type == "primitive_motion":
                return self._primitive_motion(params)
            if action_type == "stop":
                self._sdk_stop_move()
                self._update_motion_state(status="stopped", mode="idle", last_error=None)
                return "Motion stopped."
            if action_type == "damp":
                self._sdk_damp()
                self._update_motion_state(status="idle", mode="damp", last_error=None)
                return "G1 entered damp mode."
            if action_type == "squat_to_stand":
                self._sdk_damp()
                time.sleep(0.5)
                self._sdk_squat_to_stand()
                self._update_motion_state(status="idle", mode="stand", last_error=None)
                return "G1 stood up from squat."
            if action_type == "stand_to_squat":
                self._sdk_stand_to_squat()
                self._update_motion_state(status="idle", mode="squat", last_error=None)
                return "G1 moved from standing to squat."
            if action_type == "lie_to_stand":
                self._sdk_damp()
                time.sleep(0.5)
                self._sdk_lie_to_stand()
                self._update_motion_state(status="idle", mode="stand", last_error=None)
                return "G1 stood up from lying."
            if action_type == "low_stand":
                self._sdk_low_stand()
                self._update_motion_state(status="idle", mode="low_stand", last_error=None)
                return "G1 switched to low stand."
            if action_type == "high_stand":
                self._sdk_high_stand()
                self._update_motion_state(status="idle", mode="high_stand", last_error=None)
                return "G1 switched to high stand."
            if action_type == "zero_torque":
                self._sdk_zero_torque()
                self._update_motion_state(status="idle", mode="zero_torque", last_error=None)
                return "G1 entered zero torque mode."
            if action_type == "wave_hand":
                self._sdk_wave_hand(bool(params.get("turn_around", False)))
                self._update_motion_state(status="idle", mode="wave_hand", last_error=None)
                return "G1 waved hand."
            if action_type == "shake_hand":
                self._sdk_shake_hand(int(params.get("stage", -1)))
                self._update_motion_state(status="idle", mode="shake_hand", last_error=None)
                return "G1 shook hand."
            return f"Unknown action: {action_type}"
        except RuntimeError as exc:
            return f"Action failed: {exc}"

    def get_scene(self) -> dict[str, dict]:
        return dict(self._objects)

    def get_runtime_state(self) -> dict[str, Any]:
        return self._runtime_state

    def close(self) -> None:
        self.disconnect()

    def _primitive_motion(self, params: dict[str, Any]) -> str:
        primitive = str(params.get("primitive", "")).strip().lower()
        speed_scale = max(0.1, min(2.0, float(params.get("speed_scale", 1.0))))
        pose = dict(self._robot_state(self.ROBOT_ID)["robot_pose"])
        yaw_before = float(pose["yaw"])
        distance_m = float(params.get("distance_m", 0.0) or 0.0)
        angle_deg = float(params.get("angle_deg", 0.0) or 0.0)
        duration_s = params.get("duration_s")

        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        summary = primitive

        if primitive == "forward":
            command_duration_s = self._resolve_linear_duration(distance_m, duration_s, speed_scale)
            vx = self._linear_speed_mps * speed_scale
            summary = f"Moved forward for {command_duration_s:.2f}s."
        elif primitive == "backward":
            command_duration_s = self._resolve_linear_duration(distance_m, duration_s, speed_scale)
            vx = -self._linear_speed_mps * speed_scale
            summary = f"Moved backward for {command_duration_s:.2f}s."
        elif primitive == "turn_left":
            command_duration_s = self._resolve_angular_duration(angle_deg or 90.0, duration_s, speed_scale)
            vyaw = self._angular_speed_rps * speed_scale
            summary = f"Turned left for {command_duration_s:.2f}s."
        elif primitive == "turn_right":
            command_duration_s = self._resolve_angular_duration(angle_deg or 90.0, duration_s, speed_scale)
            vyaw = -self._angular_speed_rps * speed_scale
            summary = f"Turned right for {command_duration_s:.2f}s."
        elif primitive == "turn_around":
            command_duration_s = self._resolve_angular_duration(angle_deg or 180.0, duration_s, speed_scale)
            vyaw = self._angular_speed_rps * speed_scale
            summary = f"Turned around for {command_duration_s:.2f}s."
        else:
            self._update_motion_state(status="failed", mode="idle", last_error="unknown_primitive")
            return f"Unknown primitive: {primitive}"

        self._update_motion_state(
            status="executing",
            mode="primitive_motion",
            last_command={
                "primitive": primitive,
                "vx": vx,
                "vy": vy,
                "vyaw": vyaw,
                "duration_s": command_duration_s,
            },
            last_error=None,
        )

        self._sdk_set_velocity(vx, vy, vyaw, command_duration_s)
        time.sleep(command_duration_s)
        self._integrate_pose(vx=vx, vy=vy, vyaw=vyaw, duration_s=command_duration_s, yaw_before=yaw_before)
        self._update_motion_state(status="idle", mode="idle", last_error=None)
        return summary

    def _resolve_linear_duration(self, distance_m: float, duration_s: Any, speed_scale: float) -> float:
        if duration_s is not None:
            return max(self._command_dt_s, float(duration_s))
        if distance_m > 0.0:
            return max(self._command_dt_s, distance_m / max(0.05, self._linear_speed_mps * speed_scale))
        return 1.0

    def _resolve_angular_duration(self, angle_deg: float, duration_s: Any, speed_scale: float) -> float:
        if duration_s is not None:
            return max(self._command_dt_s, float(duration_s))
        angle_rad = math.radians(max(1.0, angle_deg))
        return max(self._command_dt_s, angle_rad / max(0.1, self._angular_speed_rps * speed_scale))

    def _integrate_pose(self, *, vx: float, vy: float, vyaw: float, duration_s: float, yaw_before: float) -> None:
        pose = dict(self._robot_state(self.ROBOT_ID)["robot_pose"])
        dx_body = vx * duration_s
        dy_body = vy * duration_s
        pose["x"] = float(pose["x"] + math.cos(yaw_before) * dx_body - math.sin(yaw_before) * dy_body)
        pose["y"] = float(pose["y"] + math.sin(yaw_before) * dx_body + math.cos(yaw_before) * dy_body)
        pose["yaw"] = float(pose["yaw"] + vyaw * duration_s)
        pose["stamp"] = self._stamp()
        self._robot_state(self.ROBOT_ID)["robot_pose"] = pose

    def _ensure_loco_client(self) -> None:
        if self._loco_client is not None:
            return

        sdk_root = Path(self._unitree_sdk_path).expanduser().resolve()
        if not sdk_root.exists():
            raise FileNotFoundError(f"Unitree SDK path not found: {sdk_root}")

        if str(sdk_root) not in sys.path:
            sys.path.insert(0, str(sdk_root))

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
            from unitree_sdk2py.rpc.internal import RPC_OK
        except Exception as exc:
            raise RuntimeError(f"failed to import unitree_sdk2py G1 client: {exc}") from exc

        cache_key = self._network_interface or "__default__"
        if cache_key not in _CHANNEL_INIT_CACHE:
            ChannelFactoryInitialize(0, self._network_interface or None)
            _CHANNEL_INIT_CACHE.add(cache_key)

        client = LocoClient()
        if hasattr(client, "SetTimeout"):
            client.SetTimeout(self._motion_timeout_s)
        client.Init()
        self._channel_factory_initialize = ChannelFactoryInitialize
        self._loco_client = client
        self._rpc_ok = RPC_OK
        self._safe_rpc("GetServerApiVersion")

    def _safe_rpc(self, method_name: str, *args: Any) -> Any:
        if self._loco_client is None:
            raise RuntimeError("G1 loco client is not connected")
        method = getattr(self._loco_client, method_name)
        try:
            result = method(*args)
        except Exception as exc:
            self._last_error = f"{method_name} failed: {exc}"
            self._update_motion_state(status="failed", mode="idle", last_error=self._last_error)
            raise RuntimeError(self._last_error) from exc
        if isinstance(result, tuple):
            code = result[0]
        else:
            code = result
        if code != self._rpc_ok:
            self._last_error = f"{method_name} failed with code {code}"
            self._update_motion_state(status="failed", mode="idle", last_error=self._last_error)
            raise RuntimeError(self._last_error)
        return result

    def _sdk_set_velocity(self, vx: float, vy: float, vyaw: float, duration_s: float) -> None:
        self._safe_rpc("SetVelocity", vx, vy, vyaw, duration_s)

    def _sdk_stop_move(self) -> None:
        self._sdk_set_velocity(0.0, 0.0, 0.0, 1.0)

    def _sdk_damp(self) -> None:
        self._safe_rpc("SetFsmId", 1)

    def _sdk_squat_to_stand(self) -> None:
        self._safe_rpc("SetFsmId", 706)

    def _sdk_stand_to_squat(self) -> None:
        # Mirror the current upstream G1 Python SDK wrapper.
        self._safe_rpc("SetFsmId", 706)

    def _sdk_lie_to_stand(self) -> None:
        self._safe_rpc("SetFsmId", 702)

    def _sdk_zero_torque(self) -> None:
        self._safe_rpc("SetFsmId", 0)

    def _sdk_low_stand(self) -> None:
        self._safe_rpc("SetStandHeight", self._G1_LOW_STAND_VALUE)

    def _sdk_high_stand(self) -> None:
        self._safe_rpc("SetStandHeight", self._G1_HIGH_STAND_VALUE)

    def _sdk_wave_hand(self, turn_around: bool) -> None:
        self._safe_rpc("SetTaskId", 1 if turn_around else 0)

    def _sdk_shake_hand(self, stage: int) -> None:
        if stage == 0:
            self._shake_hand_stage_state = False
            task_id = 2
        elif stage == 1:
            self._shake_hand_stage_state = True
            task_id = 3
        else:
            self._shake_hand_stage_state = not self._shake_hand_stage_state
            task_id = 3 if self._shake_hand_stage_state else 2
        self._safe_rpc("SetTaskId", task_id)

    def _robot_state(self, robot_id: str) -> dict[str, Any]:
        return self._runtime_state["robots"].setdefault(robot_id, self._make_robot_state())

    def _make_robot_state(self) -> dict[str, Any]:
        return {
            "connection_state": {
                "status": "disconnected",
                "transport": "dds",
                "host": self._network_interface or "default",
                "port": 0,
                "last_heartbeat": None,
                "last_error": None,
                "reconnect_attempts": 0,
            },
            "robot_pose": {
                "frame": "map",
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "yaw": 0.0,
                "stamp": self._stamp(),
            },
            "motion_state": {
                "mode": "idle",
                "status": "idle",
                "last_command": None,
                "last_error": None,
                "stamp": self._stamp(),
            },
        }

    def _update_motion_state(
        self,
        *,
        status: str,
        mode: str,
        last_error: str | None,
        last_command: dict[str, Any] | None = None,
    ) -> None:
        motion_state = dict(self._robot_state(self.ROBOT_ID)["motion_state"])
        motion_state.update(
            {
                "mode": mode,
                "status": status,
                "last_error": last_error,
                "stamp": self._stamp(),
            }
        )
        if last_command is not None:
            motion_state["last_command"] = last_command
        self._robot_state(self.ROBOT_ID)["motion_state"] = motion_state

    @staticmethod
    def _stamp() -> str:
        return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"
