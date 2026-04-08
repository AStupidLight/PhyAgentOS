"""Minimal models for the simplified navigation SDK."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import sys

import numpy as np

_DATACLASS_KW = {"slots": True} if sys.version_info >= (3, 10) else {}

try:
    from enum import StrEnum
except ImportError:  # pragma: no cover - Python < 3.11 fallback
    class StrEnum(str, Enum):
        def __str__(self) -> str:
            return str(self.value)


class NavPhase(StrEnum):
    IDLE = "idle"
    SEARCHING = "searching"
    TRACKING = "tracking"
    SUCCESS = "success"
    BLOCKED = "blocked"
    NOT_FOUND = "not_found"
    CANCELLED = "cancelled"


@dataclass(**_DATACLASS_KW)
class NavigationConfig:
    success_distance_m: float = 1.0
    success_heading_deg: float = 30.0
    control_mode: str = "blocking"
    timeout_s: float = 30.0


@dataclass(**_DATACLASS_KW)
class Observation:
    rgb: np.ndarray
    depth_m: np.ndarray
    occupancy: np.ndarray
    pose_xy_yaw: tuple[float, float, float]
    timestamp: float
    metadata: dict[str, object] = field(default_factory=dict)
