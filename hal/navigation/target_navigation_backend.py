"""Adapter layer that embeds navigation_sdk into OEA drivers."""

from __future__ import annotations

from dataclasses import asdict, is_dataclass
import math
from pathlib import Path
import time
from typing import Any


def _ensure_navigation_sdk_importable() -> None:
    import sys

    candidates = [
        Path(__file__).resolve().parents[2] / "navigation_sdk",
        Path(__file__).resolve().parents[3] / "navigation_sdk",
    ]
    for sdk_root in candidates:
        if sdk_root.exists():
            sdk_root_str = str(sdk_root)
            if sdk_root_str not in sys.path:
                sys.path.insert(0, sdk_root_str)
            return


def _import_navigation_sdk() -> dict[str, Any]:
    _ensure_navigation_sdk_importable()
    from navigation_mcp.bridge import (
        Go2BridgeConfig,
        Go2MoveBridge,
        SimulatedRobotBridge,
        UnitreeMujocoBridge,
    )
    from navigation_mcp.models import NavPhase, NavigationConfig, Observation
    from navigation_mcp.navigator import NavigationEngine

    return {
        "Go2BridgeConfig": Go2BridgeConfig,
        "Go2MoveBridge": Go2MoveBridge,
        "SimulatedRobotBridge": SimulatedRobotBridge,
        "UnitreeMujocoBridge": UnitreeMujocoBridge,
        "NavPhase": NavPhase,
        "NavigationConfig": NavigationConfig,
        "NavigationEngine": NavigationEngine,
        "Observation": Observation,
    }


class TargetNavigationBackend:
    """Embed the navigation_sdk engine while keeping OEA runtime semantics."""

    def __init__(self, backend_mode: str = "mock", **config: Any):
        self.backend_mode = backend_mode
        self.config = dict(config)
        self._sdk: dict[str, Any] | None = None
        self._bridge: Any = None
        self._engine: Any = None
        self._last_status: dict[str, Any] = {}
        self._connected = False

    def connect(self) -> bool:
        if self._connected:
            return True
        sdk = self._sdk_api()
        if self.backend_mode == "real":
            cfg = sdk["Go2BridgeConfig"](**self._bridge_config_kwargs())
            self._bridge = sdk["Go2MoveBridge"](cfg)
        elif self.backend_mode == "mujoco":
            cfg = sdk["Go2BridgeConfig"](**self._bridge_config_kwargs())
            self._bridge = sdk["UnitreeMujocoBridge"](cfg)
        else:
            self._bridge = sdk["SimulatedRobotBridge"]()
        self._engine = sdk["NavigationEngine"](self._bridge)
        self._connected = True
        return True

    def disconnect(self) -> None:
        if not self._connected:
            return
        bridge = self._bridge
        if bridge is not None:
            stop_remote = getattr(bridge, "stop_remote_services", None)
            if callable(stop_remote):
                try:
                    stop_remote()
                except Exception:
                    pass
            motion_server = getattr(bridge, "motion_server", None)
            if motion_server is not None:
                try:
                    motion_server.stop()
                except Exception:
                    pass
            receiver = getattr(bridge, "receiver", None)
            if receiver is not None:
                try:
                    receiver.stop()
                except Exception:
                    pass
        self._connected = False

    def health_check(self) -> dict[str, Any]:
        if not self._connected:
            return {"connected": False, "status": "disconnected"}
        if self.backend_mode in {"real", "mujoco"} and self._bridge is not None:
            describe = getattr(self._bridge, "describe", None)
            if callable(describe):
                snapshot = describe()
                return {
                    "connected": bool(snapshot.get("motion_connected")),
                    "status": "connected" if snapshot.get("motion_connected") else "degraded",
                    "details": snapshot,
                }
        return {"connected": True, "status": "connected"}

    def run_navigation(self, params: dict[str, Any]) -> dict[str, Any]:
        self.connect()
        assert self._engine is not None
        target_label = str(params.get("target_label", "")).strip()
        if not target_label:
            raise ValueError("target_label is required")
        result = self._engine.set_target(
            target_label=target_label,
            success_distance_m=params.get("success_distance_m"),
            success_heading_deg=params.get("success_heading_deg"),
            control_mode=params.get("control_mode"),
            detection_hint=params.get("detection_hint"),
        )
        timeout_s = float(params.get("timeout_s", 30.0))
        result = self._engine.run_until_done(timeout_s=timeout_s, step_delay_s=0.0)
        if result.get("phase") == "searching":
            result = {
                **result,
                "phase": "not_found",
                "message": f"target not found before timeout ({timeout_s:.1f}s)",
            }
        elif result.get("phase") == "tracking":
            result = {
                **result,
                "phase": "blocked",
                "message": f"navigation timed out while tracking target ({timeout_s:.1f}s)",
            }
        self._last_status = result
        return result

    def stop(self) -> dict[str, Any]:
        if not self._connected or self._engine is None:
            if self._bridge is not None:
                stop = getattr(self._bridge, "stop", None)
                if callable(stop):
                    stop()
            self._last_status = {"phase": "cancelled", "message": "navigation cancelled"}
            return self._last_status
        if self._bridge is not None:
            stop = getattr(self._bridge, "stop", None)
            if callable(stop):
                stop()
        self._last_status = self._engine.cancel()
        return self._last_status

    def run_motion_primitive(self, params: dict[str, Any]) -> dict[str, Any]:
        self.connect()
        if self._bridge is None:
            raise RuntimeError("motion bridge is unavailable")

        move = getattr(self._bridge, "move", None)
        stop = getattr(self._bridge, "stop", None)
        if not callable(move) or not callable(stop):
            raise RuntimeError("motion bridge does not support primitive movement")

        primitive = str(params.get("primitive", "")).strip().lower()
        if not primitive:
            raise ValueError("primitive is required")

        speed_scale = float(params.get("speed_scale", 1.0) or 1.0)
        speed_scale = min(2.0, max(0.1, speed_scale))
        dt_s = max(0.05, float(self.config.get("command_dt_s", 0.5) or 0.5))
        forward_speed = max(0.05, abs(float(self.config.get("forward_speed_x", 0.4) or 0.4))) * speed_scale
        turn_speed = max(0.05, abs(float(self.config.get("turn_speed_z", 0.6) or 0.6))) * speed_scale

        vx = 0.0
        vy = 0.0
        vyaw = 0.0
        duration_s = params.get("duration_s")

        if primitive == "forward":
            distance_m = abs(float(params.get("distance_m", 0.5) or 0.5))
            vx = forward_speed
            duration_s = float(duration_s) if duration_s is not None else distance_m / forward_speed
        elif primitive == "backward":
            distance_m = abs(float(params.get("distance_m", 0.3) or 0.3))
            vx = -forward_speed
            duration_s = float(duration_s) if duration_s is not None else distance_m / forward_speed
        elif primitive in {"turn_left", "turn_right", "turn_around"}:
            default_angle_deg = 180.0 if primitive == "turn_around" else 90.0
            angle_deg = abs(float(params.get("angle_deg", default_angle_deg) or default_angle_deg))
            vyaw = turn_speed if primitive == "turn_left" else -turn_speed
            if primitive == "turn_around":
                vyaw = turn_speed
            duration_s = float(duration_s) if duration_s is not None else math.radians(angle_deg) / turn_speed
        else:
            raise ValueError(f"unsupported primitive '{primitive}'")

        duration_s = max(dt_s, float(duration_s or dt_s))
        remaining = duration_s
        while remaining > 1e-6:
            step_s = min(dt_s, remaining)
            move(vx, vy, vyaw, duration_s=step_s)
            time.sleep(step_s)
            remaining -= step_s
        stop()

        self._last_status = {
            "phase": "success",
            "message": f"primitive motion '{primitive}' completed",
            "primitive": primitive,
            "duration_s": round(duration_s, 4),
            "command": {
                "vx": round(vx, 4),
                "vy": round(vy, 4),
                "vyaw": round(vyaw, 4),
                "speed_scale": speed_scale,
            },
        }
        return self._last_status

    def snapshot_runtime(self, robot_id: str, current_state: dict[str, Any] | None = None) -> dict[str, Any]:
        state = dict(current_state or {})
        nav_state = dict(state.get("nav_state") or {})
        status = self._last_status or (
            {} if self._engine is None else self._engine.get_status()
        )
        latest_observation = self._latest_observation()
        if latest_observation is not None:
            pose_x, pose_y, pose_yaw = latest_observation.pose_xy_yaw
            state["robot_pose"] = {
                "frame": "map",
                "x": float(pose_x),
                "y": float(pose_y),
                "z": 0.0,
                "yaw": float(pose_yaw),
                "stamp": self._timestamp(),
            }
        state["nav_state"] = {
            "mode": self._phase_to_mode(status.get("phase")),
            "status": self._phase_to_status(status.get("phase")),
            "goal_id": status.get("target_label"),
            "target_ref": None if not status.get("target_label") else {
                "kind": "target_label",
                "id": status["target_label"],
                "label": status["target_label"],
            },
            "target_label": status.get("target_label"),
            "goal": self._goal_from_status(status),
            "path_progress": self._path_progress(status),
            "recovery_count": nav_state.get("recovery_count", 0),
            "last_error": self._last_error(status),
            "relocalization_confidence": nav_state.get("relocalization_confidence"),
            "active_horizon_target": status.get("active_horizon_target"),
            "history_tail": status.get("history_tail"),
        }
        payload: dict[str, Any] = {"robots": {robot_id: state}}
        scene_graph = self._scene_graph_from_observation(latest_observation)
        if scene_graph is not None:
            payload["scene_graph"] = scene_graph
        objects = self._objects_from_observation(latest_observation)
        if objects is not None:
            payload["objects"] = objects
        return payload

    def _sdk_api(self) -> dict[str, Any]:
        if self._sdk is None:
            self._sdk = _import_navigation_sdk()
        return self._sdk

    def _bridge_config_kwargs(self) -> dict[str, Any]:
        allowed = {
            "host_bind",
            "video_port",
            "state_port",
            "occupancy_port",
            "depth_port",
            "motion_port",
            "ssh_host",
            "ssh_user",
            "ssh_password",
            "ssh_options",
            "remote_project_dir",
            "remote_python",
            "remote_setup",
            "remote_ros_choice",
            "remote_sudo_password",
            "auto_start_remote",
            "remote_livox_setup",
            "remote_livox_launch",
            "remote_data_script",
            "remote_motion_script",
            "remote_data_command",
            "remote_motion_command",
            "remote_data_video_backend",
            "remote_data_video_index",
            "remote_motion_backend",
            "remote_motion_sdk_python_path",
            "remote_motion_network_interface",
            "remote_motion_require_subscriber",
            "unitree_sdk_path",
            "network_interface",
            "enable_live_camera",
            "enable_motion_control",
            "stand_up_on_connect",
            "synthetic_depth_m",
            "command_dt_s",
            "mujoco_root",
            "mujoco_scene_path",
            "mujoco_enable_viewer",
            "mujoco_render_width",
            "mujoco_render_height",
            "mujoco_camera_fov_deg",
            "mujoco_max_depth_m",
            "mujoco_target_label",
            "mujoco_target_position_xyz",
            "mujoco_target_radius_m",
            "mujoco_target_rgb",
            "remote_sync_before_start",
            "remote_sync_paths",
            "remote_sync_excludes",
            "remote_startup_delay_s",
            "remote_observation_wait_timeout_s",
            "forward_speed_x",
            "turn_speed_z",
            "motion_confirm_timeout_s",
            "motion_confirm_translation_m",
            "motion_confirm_rotation_deg",
            "horizon_confirm_timeout_s",
            "horizon_confirm_translation_m",
            "horizon_confirm_rotation_deg",
            "lateral_speed_y",
        }
        return {key: value for key, value in self.config.items() if key in allowed}

    @staticmethod
    def _timestamp() -> str:
        from datetime import datetime

        return datetime.utcnow().replace(microsecond=0).isoformat() + "Z"

    def _latest_observation(self) -> Any:
        if self._bridge is None:
            return None
        get_observation = getattr(self._bridge, "get_observation", None)
        if not callable(get_observation):
            return None
        try:
            return get_observation()
        except Exception:
            return None

    def _scene_graph_from_observation(self, observation: Any) -> dict[str, Any] | None:
        if observation is None:
            return None
        metadata = getattr(observation, "metadata", {}) or {}
        target_label = str(metadata.get("target_label") or "").strip()
        if not target_label:
            return None
        target_xyz = metadata.get("target_position_xyz")
        if not isinstance(target_xyz, (list, tuple)) or len(target_xyz) != 3:
            return None
        radius_m = float(metadata.get("target_radius_m") or 0.12)
        return {
            "nodes": [
                {
                    "id": f"sim_{target_label}",
                    "class": target_label,
                    "object_key": target_label,
                    "center": {
                        "x": float(target_xyz[0]),
                        "y": float(target_xyz[1]),
                        "z": float(target_xyz[2]),
                    },
                    "size": {
                        "x": radius_m * 2.0,
                        "y": radius_m * 2.0,
                        "z": radius_m * 2.0,
                    },
                    "confidence": 1.0,
                    "frame": "map",
                    "track_id": f"sim_track_{target_label}",
                    "last_seen_at": self._timestamp(),
                }
            ],
            "edges": [],
        }

    def _objects_from_observation(self, observation: Any) -> dict[str, Any] | None:
        if observation is None:
            return None
        metadata = getattr(observation, "metadata", {}) or {}
        target_label = str(metadata.get("target_label") or "").strip()
        if not target_label:
            return None
        target_xyz = metadata.get("target_position_xyz")
        if not isinstance(target_xyz, (list, tuple)) or len(target_xyz) != 3:
            return None
        return {
            target_label: {
                "position": {
                    "x": float(target_xyz[0]),
                    "y": float(target_xyz[1]),
                    "z": float(target_xyz[2]),
                },
                "location": "unitree_mujoco",
            }
        }

    @staticmethod
    def _phase_to_status(phase: str | None) -> str:
        mapping = {
            "idle": "idle",
            "searching": "navigating",
            "tracking": "navigating",
            "success": "arrived",
            "blocked": "blocked",
            "not_found": "failed",
            "cancelled": "stopped",
        }
        return mapping.get(str(phase or "idle"), "idle")

    @staticmethod
    def _phase_to_mode(phase: str | None) -> str:
        mapping = {
            "idle": "idle",
            "searching": "navigating",
            "tracking": "navigating",
            "success": "navigating",
            "blocked": "navigating",
            "not_found": "navigating",
            "cancelled": "idle",
        }
        return mapping.get(str(phase or "idle"), "idle")

    def _goal_from_status(self, status: dict[str, Any]) -> dict[str, Any] | None:
        observation = self._latest_observation()
        horizon = status.get("active_horizon_target") or {}
        if observation is None and not horizon:
            return None
        pose = (0.0, 0.0, 0.0) if observation is None else observation.pose_xy_yaw
        return {
            "x": float(pose[0]),
            "y": float(pose[1]),
            "yaw": float(pose[2]),
            "horizon": horizon or None,
        }

    @staticmethod
    def _path_progress(status: dict[str, Any]) -> float | None:
        phase = status.get("phase")
        if phase == "success":
            return 1.0
        if phase in {"searching", "tracking"}:
            return min(0.99, max(0.0, float(status.get("steps", 0)) / 10.0))
        if phase == "blocked":
            return 0.5
        return None

    @staticmethod
    def _last_error(status: dict[str, Any]) -> str | None:
        phase = status.get("phase")
        if phase == "blocked":
            return "recoverable_obstacle"
        if phase == "not_found":
            return "target_not_found"
        return None


def normalize_status_payload(payload: Any) -> dict[str, Any]:
    """Convert SDK dataclasses into plain dicts when needed."""
    if isinstance(payload, dict):
        return payload
    if is_dataclass(payload):
        return asdict(payload)
    raise TypeError(f"Unsupported payload type: {type(payload)!r}")
