"""Primitive base motion tool for simple locomotion commands."""

from __future__ import annotations

from pathlib import Path
from typing import TYPE_CHECKING, Any

from PhyAgentOS.agent.tools.base import Tool
from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool

if TYPE_CHECKING:
    from PhyAgentOS.embodiment_registry import EmbodimentRegistry


class PrimitiveMotionTool(Tool):
    """Dispatch simple quadruped locomotion primitives."""

    def __init__(self, workspace: Path, action_tool: EmbodiedActionTool, registry: EmbodimentRegistry | None = None):
        self.workspace = workspace
        self.action_tool = action_tool
        self.registry = registry

    @property
    def name(self) -> str:
        return "primitive_motion"

    @property
    def description(self) -> str:
        return (
            "Execute a simple robot locomotion primitive such as moving forward, moving backward, "
            "turning left, turning right, or turning around."
        )

    @property
    def parameters(self) -> dict[str, Any]:
        return {
            "type": "object",
            "properties": {
                "robot_id": {"type": "string"},
                "primitive": {
                    "type": "string",
                    "enum": ["forward", "backward", "turn_left", "turn_right", "turn_around"],
                },
                "distance_m": {"type": "number", "minimum": 0.05},
                "angle_deg": {"type": "number", "minimum": 1.0},
                "duration_s": {"type": "number", "minimum": 0.05},
                "speed_scale": {"type": "number", "minimum": 0.1, "maximum": 2.0},
                "reasoning": {"type": "string"},
            },
            "required": ["robot_id", "primitive", "reasoning"],
        }

    async def execute(
        self,
        robot_id: str,
        primitive: str,
        reasoning: str,
        distance_m: float | None = None,
        angle_deg: float | None = None,
        duration_s: float | None = None,
        speed_scale: float | None = None,
    ) -> str:
        primitive = primitive.strip().lower()
        if not primitive:
            return "Error: primitive is required"

        parameters: dict[str, Any] = {
            "robot_id": robot_id,
            "primitive": primitive,
        }
        if distance_m is not None:
            parameters["distance_m"] = distance_m
        if angle_deg is not None:
            parameters["angle_deg"] = angle_deg
        if duration_s is not None:
            parameters["duration_s"] = duration_s
        if speed_scale is not None:
            parameters["speed_scale"] = speed_scale

        return await self.action_tool.execute(
            action_type="primitive_motion",
            parameters=parameters,
            reasoning=reasoning,
        )
