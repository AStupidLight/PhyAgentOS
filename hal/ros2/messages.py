"""Internal ROS2-facing message models."""

from __future__ import annotations

from dataclasses import dataclass, field
import sys
from typing import Any

_DATACLASS_KW = {"slots": True} if sys.version_info >= (3, 10) else {}

@dataclass(**_DATACLASS_KW)
class RobotPose:
    frame: str
    x: float
    y: float
    z: float = 0.0
    yaw: float = 0.0
    stamp: str | None = None
    covariance: list[float] | None = None


@dataclass(**_DATACLASS_KW)
class NavGoal:
    frame: str
    x: float
    y: float
    yaw: float = 0.0
    timeout_s: int = 120
    target_ref: dict[str, Any] = field(default_factory=dict)


@dataclass(**_DATACLASS_KW)
class SemanticDetection:
    track_id: str
    label: str
    confidence: float
    mask: list[list[int]] | None = None
    bbox_2d: dict[str, float] | None = None


@dataclass(**_DATACLASS_KW)
class SceneNode:
    node_id: str
    label: str
    center: dict[str, float]
    size: dict[str, float]
    frame: str = "map"
    confidence: float = 1.0
    object_key: str | None = None
