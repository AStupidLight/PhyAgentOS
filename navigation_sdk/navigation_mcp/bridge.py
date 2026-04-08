"""Simple bridges for the in-repo navigation SDK."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from pathlib import Path
import sys
import tempfile
import time
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from .models import Observation

_DATACLASS_KW = {"slots": True} if sys.version_info >= (3, 10) else {}

_DEFAULT_IMAGE_SHAPE = (64, 64, 3)
_DEFAULT_OCCUPANCY_SHAPE = (32, 32)
_TARGET_PATCH = (slice(20, 44), slice(20, 44))


def _default_unitree_sdk_path() -> str:
    candidate = Path(__file__).resolve().parents[3] / "unitree_sdk2_python"
    return str(candidate) if candidate.exists() else ""


def _default_unitree_mujoco_path() -> str:
    candidate = Path(__file__).resolve().parents[3] / "unitree_mujoco"
    return str(candidate) if candidate.exists() else ""


@dataclass(**_DATACLASS_KW)
class Go2BridgeConfig:
    host_bind: str = "127.0.0.1"
    video_port: int = 5220
    state_port: int = 5222
    occupancy_port: int = 5223
    depth_port: int = 5224
    motion_port: int = 8000
    ssh_host: str = ""
    ssh_user: str = ""
    ssh_password: str = ""
    ssh_options: list[str] = field(default_factory=list)
    remote_project_dir: str = ""
    remote_python: str = "python3"
    remote_setup: str = ""
    remote_ros_choice: str = ""
    remote_sudo_password: str = ""
    auto_start_remote: bool = False
    remote_livox_setup: str = ""
    remote_livox_launch: str = ""
    remote_data_script: str = ""
    remote_motion_script: str = ""
    remote_data_command: str = ""
    remote_motion_command: str = ""
    remote_data_video_backend: str = "realsense"
    remote_data_video_index: int = 0
    remote_motion_backend: str = "auto"
    remote_motion_sdk_python_path: str = ""
    remote_motion_network_interface: str = ""
    remote_motion_require_subscriber: bool = False
    remote_sync_before_start: bool = False
    remote_sync_paths: list[str] = field(default_factory=list)
    remote_sync_excludes: list[str] = field(default_factory=list)
    remote_startup_delay_s: float = 0.0
    remote_observation_wait_timeout_s: float = 20.0
    forward_speed_x: float = 0.4
    lateral_speed_y: float = 0.0
    turn_speed_z: float = 0.6
    motion_confirm_timeout_s: float = 3.0
    motion_confirm_translation_m: float = 0.2
    motion_confirm_rotation_deg: float = 15.0
    horizon_confirm_timeout_s: float = 3.0
    horizon_confirm_translation_m: float = 0.2
    horizon_confirm_rotation_deg: float = 15.0
    unitree_sdk_path: str = ""
    network_interface: str = ""
    enable_live_camera: bool = True
    enable_motion_control: bool = True
    stand_up_on_connect: bool = False
    synthetic_depth_m: float = 2.0
    command_dt_s: float = 0.5
    mujoco_root: str = ""
    mujoco_scene_path: str = ""
    mujoco_enable_viewer: bool = False
    mujoco_render_width: int = 96
    mujoco_render_height: int = 96
    mujoco_camera_fov_deg: float = 70.0
    mujoco_max_depth_m: float = 8.0
    mujoco_target_label: str = "auto"
    mujoco_target_position_xyz: tuple[float, float, float] = (3.4, 0.0, 0.92)
    mujoco_target_radius_m: float = 0.4
    mujoco_target_rgb: tuple[int, int, int] = (220, 20, 20)


class SimulatedRobotBridge:
    """Tiny bridge that exposes just enough surface for the current Go2 pipeline."""

    def __init__(self) -> None:
        self.pose_xy_yaw = (0.0, 0.0, 0.0)
        self.obstacle_cells: set[tuple[int, int]] = set()
        self.motion_connected = True
        self.camera_connected = True
        self._target_color = np.array([220, 20, 20], dtype=np.uint8)
        self._synthetic_depth_m = 2.0
        self._last_move_command = (0.0, 0.0, 0.0)

    def get_observation(self) -> Observation:
        rgb = np.zeros(_DEFAULT_IMAGE_SHAPE, dtype=np.uint8)
        rgb[_TARGET_PATCH] = self._target_color
        depth = np.full(_DEFAULT_IMAGE_SHAPE[:2], np.nan, dtype=np.float32)
        occupancy = np.zeros(_DEFAULT_OCCUPANCY_SHAPE, dtype=np.uint8)
        for x, y in self.obstacle_cells:
            if 0 <= y < occupancy.shape[0] and 0 <= x < occupancy.shape[1]:
                occupancy[y, x] = 1
        return Observation(
            rgb=rgb,
            depth_m=depth,
            occupancy=occupancy,
            pose_xy_yaw=self.pose_xy_yaw,
            timestamp=time.time(),
        )

    def move(self, vx: float, vy: float, vyaw: float, duration_s: float = 0.5) -> None:
        x, y, yaw = self.pose_xy_yaw
        self.pose_xy_yaw = (
            float(x + vx * duration_s),
            float(y + vy * duration_s),
            float(yaw + vyaw * duration_s),
        )
        self._last_move_command = (float(vx), float(vy), float(vyaw))

    def stop(self) -> None:
        self._last_move_command = (0.0, 0.0, 0.0)

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        self.pose_xy_yaw = (float(x), float(y), float(yaw))

    def stop_remote_services(self) -> None:
        self.motion_connected = False
        self.camera_connected = False

    def describe(self) -> dict[str, object]:
        return {
            "motion_connected": self.motion_connected,
            "camera_connected": self.camera_connected,
            "pose_xy_yaw": self.pose_xy_yaw,
            "obstacle_cells": sorted(self.obstacle_cells),
            "last_move_command": self._last_move_command,
        }


class UnitreeMujocoBridge(SimulatedRobotBridge):
    """Embed the Unitree MuJoCo scene as a lightweight navigation sandbox."""

    _STAND_JOINTS = {
        "FR_hip_joint": 0.00571868,
        "FR_thigh_joint": 0.608813,
        "FR_calf_joint": -1.21763,
        "FL_hip_joint": -0.00571868,
        "FL_thigh_joint": 0.608813,
        "FL_calf_joint": -1.21763,
        "RR_hip_joint": 0.00571868,
        "RR_thigh_joint": 0.608813,
        "RR_calf_joint": -1.21763,
        "RL_hip_joint": -0.00571868,
        "RL_thigh_joint": 0.608813,
        "RL_calf_joint": -1.21763,
    }

    def __init__(self, config: Go2BridgeConfig | dict[str, Any]):
        super().__init__()
        self.config = config if isinstance(config, Go2BridgeConfig) else Go2BridgeConfig(**config)
        self._mujoco: Any = None
        self._model: Any = None
        self._data: Any = None
        self._viewer: Any = None
        self._temp_scene_path: str | None = None
        self._base_height_m = 0.445
        self._target_position = np.asarray(self.config.mujoco_target_position_xyz, dtype=np.float32)
        self._target_radius_m = float(self.config.mujoco_target_radius_m)
        self._target_color = np.asarray(self.config.mujoco_target_rgb, dtype=np.uint8)
        self._scene_features: list[dict[str, Any]] = []
        self._unitree_error: str | None = None
        self._mujoco_ready = False
        self._initialize_sim()

    def get_observation(self) -> Observation:
        if not self._mujoco_ready:
            return super().get_observation()

        active_feature = self._active_target_feature()
        rgb, depth = self._render_virtual_camera()
        occupancy = np.zeros(_DEFAULT_OCCUPANCY_SHAPE, dtype=np.uint8)
        self._sync_pose_from_sim()
        self._sync_viewer()
        valid_depth = np.isfinite(depth)
        visible_distance_m = float(np.nanmedian(depth[valid_depth])) if np.any(valid_depth) else None
        target_label = str(active_feature.get("label", self.config.mujoco_target_label)) if active_feature else str(self.config.mujoco_target_label)
        target_position = (
            [float(v) for v in active_feature.get("center_xyz", self._target_position)]
            if active_feature
            else [float(v) for v in self._target_position]
        )
        target_radius = float(active_feature.get("radius_m", self._target_radius_m)) if active_feature else float(self._target_radius_m)
        return Observation(
            rgb=rgb,
            depth_m=depth,
            occupancy=occupancy,
            pose_xy_yaw=self.pose_xy_yaw,
            timestamp=time.time(),
            metadata={
                "source": "unitree_mujoco",
                "scene_path": self._scene_path(),
                "target_label": target_label,
                "target_position_xyz": target_position,
                "target_radius_m": target_radius,
                "visible_target_label": target_label if visible_distance_m is not None else None,
                "visible_target_distance_m": visible_distance_m,
                "scene_features": self._scene_features,
            },
        )

    def move(self, vx: float, vy: float, vyaw: float, duration_s: float = 0.5) -> None:
        duration_s = max(0.0, float(duration_s))
        super().move(vx, vy, vyaw, duration_s)
        if not self._mujoco_ready:
            return

        x, y, yaw = self.pose_xy_yaw
        self._set_base_pose(x=x, y=y, yaw=yaw)
        self._sync_viewer()

    def stop_remote_services(self) -> None:
        self.stop()
        if self._viewer is not None:
            close = getattr(self._viewer, "close", None)
            if callable(close):
                try:
                    close()
                except Exception:
                    pass
        self.motion_connected = False
        self.camera_connected = False

    def describe(self) -> dict[str, object]:
        base = super().describe()
        active_feature = self._active_target_feature()
        base.update(
            {
                "simulation_backend": "unitree_mujoco",
                "scene_path": self._scene_path(),
                "mujoco_ready": self._mujoco_ready,
                "viewer_enabled": bool(self.config.mujoco_enable_viewer),
                "target_label": (active_feature or {}).get("label", self.config.mujoco_target_label),
                "target_position_xyz": [float(v) for v in (active_feature or {}).get("center_xyz", self._target_position)],
                "target_radius_m": float((active_feature or {}).get("radius_m", self._target_radius_m)),
                "scene_features": self._scene_features,
                "unitree_error": self._unitree_error,
            }
        )
        return base

    def _initialize_sim(self) -> None:
        try:
            import mujoco
        except Exception as exc:
            self._unitree_error = f"failed to import mujoco: {exc}"
            self.motion_connected = False
            self.camera_connected = False
            return

        self._mujoco = mujoco
        self._scene_features = self._extract_scene_features()
        active_feature = self._active_target_feature()
        if active_feature is not None:
            self._target_position = np.asarray(active_feature["center_xyz"], dtype=np.float32)
            self._target_radius_m = float(active_feature["radius_m"])
        scene_path = self._prepare_scene()
        if scene_path is None:
            self.motion_connected = False
            self.camera_connected = False
            return

        try:
            self._model = mujoco.MjModel.from_xml_path(scene_path)
            self._data = mujoco.MjData(self._model)
            self._apply_stand_pose()
            self._set_base_pose(x=0.0, y=0.0, yaw=0.0)
            if self.config.mujoco_enable_viewer:
                import mujoco.viewer

                self._viewer = mujoco.viewer.launch_passive(self._model, self._data)
            self.motion_connected = True
            self.camera_connected = True
            self._mujoco_ready = True
        except Exception as exc:
            self._unitree_error = f"failed to init unitree_mujoco scene: {exc}"
            self.motion_connected = False
            self.camera_connected = False

    def _scene_path(self) -> str:
        if self.config.mujoco_scene_path:
            return self.config.mujoco_scene_path
        mujoco_root = Path(self.config.mujoco_root or _default_unitree_mujoco_path()).expanduser()
        return str((mujoco_root / "unitree_robots" / "go2" / "scene.xml").resolve())

    def _extract_scene_features(self) -> list[dict[str, Any]]:
        scene_path = Path(self._scene_path()).expanduser().resolve()
        if not scene_path.exists():
            return []
        try:
            tree = ET.parse(scene_path)
            root = tree.getroot()
            worldbody = root.find("worldbody")
            if worldbody is None:
                return []
            boxes: list[dict[str, float]] = []
            for geom in worldbody.findall("geom"):
                if geom.attrib.get("type", "").strip() != "box":
                    continue
                pos = [float(v) for v in geom.attrib.get("pos", "0 0 0").split()]
                size = [float(v) for v in geom.attrib.get("size", "0 0 0").split()]
                if len(pos) != 3 or len(size) != 3:
                    continue
                boxes.append(
                    {
                        "x": pos[0],
                        "y": pos[1],
                        "z": pos[2],
                        "sx": size[0],
                        "sy": size[1],
                        "sz": size[2],
                    }
                )
            stair_boxes = [box for box in boxes if box["sx"] >= 0.19 and box["sy"] >= 1.5 and box["sz"] >= 0.15]
            if len(stair_boxes) < 3:
                return []
            x_min = min(box["x"] - box["sx"] for box in stair_boxes)
            x_max = max(box["x"] + box["sx"] for box in stair_boxes)
            y_min = min(box["y"] - box["sy"] for box in stair_boxes)
            y_max = max(box["y"] + box["sy"] for box in stair_boxes)
            z_max = max(box["z"] + box["sz"] for box in stair_boxes)
            center_xyz = (
                (x_min + x_max) / 2.0,
                (y_min + y_max) / 2.0,
                z_max / 2.0,
            )
            span_x = x_max - x_min
            span_y = y_max - y_min
            span_z = z_max
            return [
                {
                    "label": "stairs",
                    "center_xyz": center_xyz,
                    "size_xyz": (span_x, span_y, span_z),
                    "radius_m": max(span_x, span_y, span_z) / 2.0,
                }
            ]
        except Exception:
            return []

    def _active_target_feature(self) -> dict[str, Any] | None:
        target_label = str(self.config.mujoco_target_label or "").strip().lower()
        if target_label and target_label != "auto":
            for feature in self._scene_features:
                if str(feature.get("label", "")).lower() == target_label:
                    return feature
            return {
                "label": self.config.mujoco_target_label,
                "center_xyz": tuple(float(v) for v in self._target_position),
                "radius_m": float(self._target_radius_m),
            }
        return self._scene_features[0] if self._scene_features else None

    def _prepare_scene(self) -> str | None:
        scene_path = Path(self._scene_path()).expanduser().resolve()
        if not scene_path.exists():
            self._unitree_error = f"unitree_mujoco scene not found: {scene_path}"
            return None

        try:
            workspace_dir = Path(tempfile.mkdtemp(prefix="phyagentos_go2_mujoco_"))
            go2_path = (scene_path.parent / "go2.xml").resolve()
            if not go2_path.exists():
                self._unitree_error = f"unitree_mujoco robot model not found: {go2_path}"
                return None

            go2_tree = ET.parse(go2_path)
            go2_root = go2_tree.getroot()
            compiler = go2_root.find("compiler")
            if compiler is not None:
                compiler.set("meshdir", str((go2_path.parent / "assets").resolve()))
            patched_go2_path = workspace_dir / "go2.xml"
            go2_tree.write(patched_go2_path, encoding="utf-8", xml_declaration=False)

            tree = ET.parse(scene_path)
            root = tree.getroot()
            include = root.find("include")
            if include is not None:
                include.set("file", str(patched_go2_path))
            worldbody = root.find("worldbody")
            if worldbody is None:
                self._unitree_error = "unitree_mujoco scene missing <worldbody>"
                return None

            target_body = ET.SubElement(
                worldbody,
                "body",
                {
                    "name": "track_a_target",
                    "pos": " ".join(f"{float(v):.4f}" for v in self._target_position),
                },
            )
            rgba = np.clip(self._target_color.astype(np.float32) / 255.0, 0.0, 1.0)
            ET.SubElement(
                target_body,
                "geom",
                {
                    "name": "track_a_target_geom",
                    "type": "sphere",
                    "size": f"{self._target_radius_m:.4f}",
                    "rgba": f"{rgba[0]:.4f} {rgba[1]:.4f} {rgba[2]:.4f} 1",
                    "contype": "0",
                    "conaffinity": "0",
                },
            )
            ET.SubElement(
                worldbody,
                "camera",
                {
                    "name": "track_a_overview",
                    "pos": "0 -3.5 1.8",
                    "xyaxes": "1 0 0 0 0 1",
                },
            )

            patched_scene_path = workspace_dir / "scene.xml"
            tree.write(patched_scene_path, encoding="utf-8", xml_declaration=False)
            self._temp_scene_path = str(patched_scene_path)
            return str(patched_scene_path)
        except Exception as exc:
            self._unitree_error = f"failed to patch unitree_mujoco scene: {exc}"
            return None

    def _apply_stand_pose(self) -> None:
        if self._model is None or self._data is None or self._mujoco is None:
            return
        for joint_name, joint_pos in self._STAND_JOINTS.items():
            try:
                joint_id = self._mujoco.mj_name2id(
                    self._model, self._mujoco.mjtObj.mjOBJ_JOINT, joint_name
                )
            except Exception:
                continue
            if joint_id < 0:
                continue
            qpos_adr = int(self._model.jnt_qposadr[joint_id])
            self._data.qpos[qpos_adr] = float(joint_pos)

    def _set_base_pose(self, *, x: float, y: float, yaw: float) -> None:
        if self._data is None or self._mujoco is None:
            return
        self._data.qpos[0] = float(x)
        self._data.qpos[1] = float(y)
        self._data.qpos[2] = float(self._base_height_m)
        quat = self._yaw_to_quat(float(yaw))
        self._data.qpos[3:7] = quat
        self._data.qvel[:6] = 0.0
        self._mujoco.mj_forward(self._model, self._data)
        self.pose_xy_yaw = (float(x), float(y), float(yaw))

    def _sync_pose_from_sim(self) -> None:
        if self._data is None:
            return
        x = float(self._data.qpos[0])
        y = float(self._data.qpos[1])
        qw, qx, qy, qz = [float(v) for v in self._data.qpos[3:7]]
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        self.pose_xy_yaw = (x, y, yaw)

    def _render_virtual_camera(self) -> tuple[np.ndarray, np.ndarray]:
        height = max(32, int(self.config.mujoco_render_height))
        width = max(32, int(self.config.mujoco_render_width))
        rgb = np.zeros((height, width, 3), dtype=np.uint8)
        rgb[..., 1] = 24
        rgb[..., 2] = 48
        depth = np.full((height, width), np.nan, dtype=np.float32)

        active_feature = self._active_target_feature()
        if active_feature is not None:
            target_position = np.asarray(active_feature.get("center_xyz", self._target_position), dtype=np.float32)
            target_radius = float(active_feature.get("radius_m", self._target_radius_m))
        else:
            target_position = self._target_position
            target_radius = float(self._target_radius_m)

        x, y, yaw = self.pose_xy_yaw
        dx = float(target_position[0] - x)
        dy = float(target_position[1] - y)
        rel_forward = math.cos(yaw) * dx + math.sin(yaw) * dy
        rel_left = -math.sin(yaw) * dx + math.cos(yaw) * dy
        planar_distance = math.hypot(dx, dy)
        max_depth = max(1.0, float(self.config.mujoco_max_depth_m))

        if rel_forward <= 0.05 or planar_distance > max_depth:
            return rgb, depth

        half_fov = math.radians(float(self.config.mujoco_camera_fov_deg)) / 2.0
        bearing = math.atan2(rel_left, rel_forward)
        if abs(bearing) > half_fov:
            return rgb, depth

        focal = (width / 2.0) / math.tan(max(0.05, half_fov))
        cx = int(round(width / 2.0 - math.tan(bearing) * focal))
        cy = height // 2
        radius_px = max(4, int(round(focal * target_radius / max(planar_distance, 0.1))))
        yy, xx = np.ogrid[:height, :width]
        mask = (xx - cx) ** 2 + (yy - cy) ** 2 <= radius_px**2
        if not np.any(mask):
            return rgb, depth

        rgb[mask] = self._target_color
        depth[mask] = np.float32(planar_distance)
        return rgb, depth

    def _sync_viewer(self) -> None:
        if self._viewer is None:
            return
        is_running = getattr(self._viewer, "is_running", None)
        if callable(is_running) and not is_running():
            return
        sync = getattr(self._viewer, "sync", None)
        if callable(sync):
            try:
                sync()
            except Exception:
                pass

    @staticmethod
    def _yaw_to_quat(yaw: float) -> np.ndarray:
        half = yaw / 2.0
        return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=np.float64)


class Go2MoveBridge(SimulatedRobotBridge):
    """Hybrid bridge: use Unitree SDK when available, otherwise fall back to simulation."""

    def __init__(self, config: Go2BridgeConfig | dict[str, Any]):
        super().__init__()
        self.motion_connected = False
        self.camera_connected = False
        self.config = config if isinstance(config, Go2BridgeConfig) else Go2BridgeConfig(**config)
        self._synthetic_depth_m = float(self.config.synthetic_depth_m)
        self._unitree_available = False
        self._unitree_error: str | None = None
        self._video_client: Any = None
        self._sport_client: Any = None
        self._cv2: Any = None
        self._channel_initialized = False
        self._live_frame_shape: tuple[int, int] | None = None
        self._channel_factory_initialize = None
        self._initialize_unitree_clients()

    def get_observation(self) -> Observation:
        frame = self._read_live_frame()
        if frame is None:
            return self._simulated_observation()

        height, width = frame.shape[:2]
        depth = np.full((height, width), self._synthetic_depth_m, dtype=np.float32)
        occupancy = np.zeros(_DEFAULT_OCCUPANCY_SHAPE, dtype=np.uint8)
        return Observation(
            rgb=frame,
            depth_m=depth,
            occupancy=occupancy,
            pose_xy_yaw=self.pose_xy_yaw,
            timestamp=time.time(),
            metadata={"source": "unitree_camera"},
        )

    def move(self, vx: float, vy: float, vyaw: float, duration_s: float = 0.5) -> None:
        duration_s = max(0.0, float(duration_s))
        super().move(vx, vy, vyaw, duration_s)
        self._synthetic_depth_m = max(0.3, self._synthetic_depth_m - abs(vx) * duration_s)

        if not self._sport_client or not self.enable_motion_control:
            return
        try:
            self._sport_client.Move(float(vx), float(vy), float(vyaw))
        except Exception as exc:
            self.motion_connected = False
            self._unitree_error = f"sport move failed: {exc}"

    def stop(self) -> None:
        super().stop()
        if not self._sport_client or not self.enable_motion_control:
            return
        try:
            self._sport_client.StopMove()
        except Exception as exc:
            self.motion_connected = False
            self._unitree_error = f"sport stop failed: {exc}"

    def stop_remote_services(self) -> None:
        self.stop()
        super().stop_remote_services()

    @property
    def enable_live_camera(self) -> bool:
        return bool(self.config.enable_live_camera)

    @property
    def enable_motion_control(self) -> bool:
        return bool(self.config.enable_motion_control)

    def describe(self) -> dict[str, object]:
        base = super().describe()
        base.update(
            {
                "ssh_host": self.config.ssh_host,
                "video_port": self.config.video_port,
                "depth_port": self.config.depth_port,
                "state_port": self.config.state_port,
                "motion_port": self.config.motion_port,
                "video_backend": self.config.remote_data_video_backend,
                "network_interface": self._network_interface(),
                "unitree_sdk_path": self._sdk_path(),
                "unitree_available": self._unitree_available,
                "unitree_error": self._unitree_error,
                "live_frame_shape": self._live_frame_shape,
                "synthetic_depth_m": self._synthetic_depth_m,
            }
        )
        return base

    def _simulated_observation(self) -> Observation:
        rgb = np.zeros(_DEFAULT_IMAGE_SHAPE, dtype=np.uint8)
        rgb[_TARGET_PATCH] = self._target_color
        depth = np.full(_DEFAULT_IMAGE_SHAPE[:2], self._synthetic_depth_m, dtype=np.float32)
        occupancy = np.zeros(_DEFAULT_OCCUPANCY_SHAPE, dtype=np.uint8)
        for x, y in self.obstacle_cells:
            if 0 <= y < occupancy.shape[0] and 0 <= x < occupancy.shape[1]:
                occupancy[y, x] = 1
        return Observation(
            rgb=rgb,
            depth_m=depth,
            occupancy=occupancy,
            pose_xy_yaw=self.pose_xy_yaw,
            timestamp=time.time(),
            metadata={"source": "simulated"},
        )

    def _sdk_path(self) -> str:
        return (
            self.config.unitree_sdk_path
            or self.config.remote_motion_sdk_python_path
            or _default_unitree_sdk_path()
        )

    def _network_interface(self) -> str | None:
        return self.config.network_interface or self.config.remote_motion_network_interface or None

    def _initialize_unitree_clients(self) -> None:
        sdk_path = self._sdk_path()
        if not sdk_path:
            self._unitree_error = "unitree_sdk_path not configured"
            return

        sdk_root = Path(sdk_path).expanduser().resolve()
        if not sdk_root.exists():
            self._unitree_error = f"Unitree SDK path not found: {sdk_root}"
            return

        if str(sdk_root) not in sys.path:
            sys.path.insert(0, str(sdk_root))

        try:
            from unitree_sdk2py.core.channel import ChannelFactoryInitialize
            from unitree_sdk2py.go2.sport.sport_client import SportClient
            from unitree_sdk2py.go2.video.video_client import VideoClient
            import cv2
        except Exception as exc:
            self._unitree_error = f"failed to import unitree_sdk2py: {exc}"
            return

        self._channel_factory_initialize = ChannelFactoryInitialize
        self._cv2 = cv2

        try:
            self._ensure_channel_initialized()
            if self.enable_live_camera:
                self._video_client = VideoClient()
                self._video_client.SetTimeout(float(self.config.remote_observation_wait_timeout_s))
                self._video_client.Init()
                self.camera_connected = True
            if self.enable_motion_control:
                self._sport_client = SportClient()
                self._sport_client.SetTimeout(float(self.config.motion_confirm_timeout_s or 3.0))
                self._sport_client.Init()
                self.motion_connected = True
                if self.config.stand_up_on_connect:
                    self._sport_client.StandUp()
            self._unitree_available = True
        except Exception as exc:
            self._video_client = None
            self._sport_client = None
            self.motion_connected = False
            self.camera_connected = False
            self._unitree_error = f"failed to init unitree clients: {exc}"

    def _ensure_channel_initialized(self) -> None:
        if self._channel_initialized or self._channel_factory_initialize is None:
            return
        interface = self._network_interface()
        self._channel_factory_initialize(0, interface)
        self._channel_initialized = True

    def _read_live_frame(self) -> np.ndarray | None:
        if not self._video_client or not self.enable_live_camera or self._cv2 is None:
            return None
        try:
            code, data = self._video_client.GetImageSample()
            if code != 0 or not data:
                return None
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            frame = self._cv2.imdecode(image_data, self._cv2.IMREAD_COLOR)
            if frame is None:
                return None
            self._live_frame_shape = frame.shape[:2]
            return frame
        except Exception as exc:
            self.camera_connected = False
            self._unitree_error = f"video read failed: {exc}"
            return None
