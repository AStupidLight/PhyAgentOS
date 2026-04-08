from __future__ import annotations

import asyncio
import json
from pathlib import Path

from PhyAgentOS.agent.tools.embodied import EmbodiedActionTool
from PhyAgentOS.agent.tools.primitive_motion import PrimitiveMotionTool
from hal.drivers import load_driver
from hal.hal_watchdog import _poll_once
from hal.simulation.scene_io import load_environment_doc, save_environment_doc

_FENCE_OPEN = "```json"
_FENCE_CLOSE = "```"


class _FakeResponse:
    def __init__(self, content: str):
        self.content = content


class _FakeProvider:
    async def chat_with_retry(self, messages, model):  # noqa: ANN001
        return _FakeResponse("VALID")


def _write_workspace_files(workspace: Path) -> None:
    (workspace / "EMBODIED.md").write_text(
        "# Embodied\n\n- Supports primitive motion.\n",
        encoding="utf-8",
    )
    (workspace / "LESSONS.md").write_text("# Lessons\n", encoding="utf-8")
    save_environment_doc(
        workspace / "ENVIRONMENT.md",
        {
            "schema_version": "PhyAgentOS.environment.v1",
            "scene_graph": {"nodes": [], "edges": []},
            "robots": {
                "go2_edu_001": {
                    "robot_pose": {
                        "frame": "map",
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "yaw": 0.0,
                        "stamp": "2026-04-08T00:00:00Z",
                    },
                    "nav_state": {
                        "mode": "idle",
                        "status": "idle",
                        "recovery_count": 0,
                    },
                }
            },
            "objects": {},
        },
    )


def _write_action(path: Path, payload: dict) -> Path:
    action_file = path / "ACTION.md"
    action_file.write_text(
        f"{_FENCE_OPEN}\n{json.dumps(payload, indent=2)}\n{_FENCE_CLOSE}\n",
        encoding="utf-8",
    )
    return action_file


def test_primitive_motion_writes_action_md(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    action_tool = EmbodiedActionTool(workspace=tmp_path, provider=_FakeProvider(), model="fake")
    tool = PrimitiveMotionTool(workspace=tmp_path, action_tool=action_tool)

    result = asyncio.run(
        tool.execute(
            robot_id="go2_edu_001",
            primitive="forward",
            distance_m=0.6,
            reasoning="Need to move the robot a short distance forward.",
        )
    )

    assert "validated and dispatched" in result
    action_doc = (tmp_path / "ACTION.md").read_text(encoding="utf-8")
    assert "primitive_motion" in action_doc
    assert '"primitive": "forward"' in action_doc


def test_go2_driver_primitive_motion_forward_updates_pose(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    env_file = tmp_path / "ENVIRONMENT.md"
    action_file = _write_action(
        tmp_path,
        {
            "action_type": "primitive_motion",
            "parameters": {
                "robot_id": "go2_edu_001",
                "primitive": "forward",
                "distance_m": 0.8,
            },
            "status": "pending",
        },
    )

    with load_driver("go2_edu", gui=False, target_navigation_backend="mock", command_dt_s=0.05) as driver:
        driver.connect()
        _poll_once(driver, action_file, env_file)

    updated = load_environment_doc(env_file)
    pose = updated["robots"]["go2_edu_001"]["robot_pose"]
    assert pose["x"] > 0.6
    assert updated["robots"]["go2_edu_001"]["nav_state"]["status"] == "idle"


def test_go2_driver_primitive_motion_turn_around_updates_yaw(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)
    env_file = tmp_path / "ENVIRONMENT.md"
    action_file = _write_action(
        tmp_path,
        {
            "action_type": "primitive_motion",
            "parameters": {
                "robot_id": "go2_edu_001",
                "primitive": "turn_around",
            },
            "status": "pending",
        },
    )

    with load_driver("go2_edu", gui=False, target_navigation_backend="mock", command_dt_s=0.05) as driver:
        driver.connect()
        _poll_once(driver, action_file, env_file)

    updated = load_environment_doc(env_file)
    yaw = updated["robots"]["go2_edu_001"]["robot_pose"]["yaw"]
    assert yaw > 2.5
