"""Simple target-navigation engine compatible with the current Go2 backend."""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from .models import NavPhase, NavigationConfig, Observation


class NavigationEngine:
    """Minimal navigation engine for target-label navigation smoke tests."""

    def __init__(self, bridge: Any):
        self.bridge = bridge
        self._config = NavigationConfig()
        self._target_label = ""
        self._detection_hint: dict[str, Any] | None = None
        self._history: list[str] = []
        self._status: dict[str, Any] = self._make_status(NavPhase.IDLE, message="idle")

    def set_target(
        self,
        *,
        target_label: str,
        success_distance_m: float | None = None,
        success_heading_deg: float | None = None,
        control_mode: str | None = None,
        detection_hint: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        self._target_label = target_label
        self._detection_hint = dict(detection_hint or {})
        self._config = NavigationConfig(
            success_distance_m=float(success_distance_m or 1.0),
            success_heading_deg=float(success_heading_deg or 30.0),
            control_mode=str(control_mode or "blocking"),
        )
        self._status = self._make_status(
            NavPhase.SEARCHING,
            message=f"searching for {target_label}",
            target_label=target_label,
            detection_hint=self._detection_hint or None,
        )
        return dict(self._status)

    def run_until_done(self, *, timeout_s: float, step_delay_s: float = 0.0) -> dict[str, Any]:
        timeout_s = max(0.1, float(timeout_s))
        step_delay_s = max(0.0, float(step_delay_s))
        max_steps = max(1, int(math.ceil(timeout_s / max(0.1, getattr(self.bridge, "config", None).command_dt_s if getattr(self.bridge, "config", None) else 0.5))))

        last_distance: float | None = None
        for step in range(1, max_steps + 1):
            observation = self.bridge.get_observation()

            if getattr(self.bridge, "obstacle_cells", None):
                self.bridge.stop()
                self._status = self._make_status(
                    NavPhase.BLOCKED,
                    message=f"blocked while moving toward {self._target_label}",
                    target_label=self._target_label,
                    steps=step,
                    active_horizon_target={"label": self._target_label},
                )
                return dict(self._status)

            if not self._target_label or not self._has_target_detection(observation):
                self.bridge.stop()
                self._status = self._make_status(
                    NavPhase.SEARCHING,
                    message=f"target {self._target_label or 'unknown'} not detected",
                    target_label=self._target_label or None,
                    steps=step,
                )
                return dict(self._status)

            distance_m = self._estimate_distance_m(observation)
            last_distance = distance_m
            if distance_m <= self._config.success_distance_m:
                self.bridge.stop()
                self._status = self._make_status(
                    NavPhase.SUCCESS,
                    message=f"arrived near {self._target_label}",
                    target_label=self._target_label,
                    steps=step,
                    active_horizon_target={"label": self._target_label, "distance_m": distance_m},
                )
                return dict(self._status)

            self._status = self._make_status(
                NavPhase.TRACKING,
                message=f"tracking {self._target_label}",
                target_label=self._target_label,
                steps=step,
                active_horizon_target={"label": self._target_label, "distance_m": distance_m},
            )
            self.bridge.move(getattr(self.bridge, "config", None).forward_speed_x if getattr(self.bridge, "config", None) else 0.4, 0.0, 0.0, getattr(self.bridge, "config", None).command_dt_s if getattr(self.bridge, "config", None) else 0.5)
            if step_delay_s:
                import time
                time.sleep(step_delay_s)

        self.bridge.stop()
        self._status = self._make_status(
            NavPhase.BLOCKED if last_distance is not None else NavPhase.NOT_FOUND,
            message=f"navigation timed out while tracking target ({timeout_s:.1f}s)" if last_distance is not None else f"target not found before timeout ({timeout_s:.1f}s)",
            target_label=self._target_label or None,
            steps=max_steps,
            active_horizon_target=None if last_distance is None else {"label": self._target_label, "distance_m": last_distance},
        )
        return dict(self._status)

    def get_status(self) -> dict[str, Any]:
        return dict(self._status)

    def cancel(self) -> dict[str, Any]:
        self._status = self._make_status(
            NavPhase.CANCELLED,
            message=f"navigation cancelled for {self._target_label or 'target'}",
            target_label=self._target_label or None,
        )
        return dict(self._status)

    def _has_target_detection(self, observation: Observation) -> bool:
        rgb_range = self._detection_hint.get("rgb_range") if self._detection_hint else None
        metadata_label = str(observation.metadata.get("visible_target_label") or "")
        if not rgb_range:
            depth = observation.depth_m
            return (
                bool(self._target_label)
                and metadata_label == self._target_label
                and depth is not None
                and bool(np.any(np.isfinite(depth)))
            )
        if not rgb_range or len(rgb_range) != 2:
            return False
        lower = np.array(rgb_range[0], dtype=np.uint8)
        upper = np.array(rgb_range[1], dtype=np.uint8)
        rgb_mask = np.all(observation.rgb >= lower, axis=2) & np.all(observation.rgb <= upper, axis=2)
        if not np.any(rgb_mask):
            return False
        depth = observation.depth_m
        if depth is None:
            return False
        valid_depth = np.isfinite(depth) & rgb_mask
        if not np.any(valid_depth):
            return False
        return True

    def _estimate_distance_m(self, observation: Observation) -> float:
        rgb_range = self._detection_hint.get("rgb_range") if self._detection_hint else None
        metadata_label = str(observation.metadata.get("visible_target_label") or "")
        metadata_distance = observation.metadata.get("visible_target_distance_m")
        if not rgb_range:
            if metadata_label == self._target_label and metadata_distance is not None:
                return float(metadata_distance)
            return self._config.success_distance_m
        if not rgb_range or len(rgb_range) != 2:
            return self._config.success_distance_m
        lower = np.array(rgb_range[0], dtype=np.uint8)
        upper = np.array(rgb_range[1], dtype=np.uint8)
        rgb_mask = np.all(observation.rgb >= lower, axis=2) & np.all(observation.rgb <= upper, axis=2)
        depth = observation.depth_m
        valid_depth = np.isfinite(depth) & rgb_mask
        if not np.any(valid_depth):
            return self._config.success_distance_m
        return float(np.nanmedian(depth[valid_depth]))

    def _make_status(self, phase: NavPhase, **extra: Any) -> dict[str, Any]:
        phase_value = str(phase)
        self._history.append(phase_value)
        history_tail = self._history[-5:]
        return {
            "phase": phase_value,
            "target_label": extra.pop("target_label", None),
            "message": extra.pop("message", phase_value),
            "steps": int(extra.pop("steps", 0)),
            "active_horizon_target": extra.pop("active_horizon_target", None),
            "history_tail": history_tail,
            **extra,
        }
