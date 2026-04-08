from __future__ import annotations

import asyncio
from pathlib import Path
from unittest.mock import AsyncMock, MagicMock

from PhyAgentOS.agent.loop import AgentLoop
from PhyAgentOS.bus.queue import MessageBus
from hal.simulation.scene_io import save_environment_doc


class _FakeResponse:
    def __init__(self, content: str):
        self.content = content


def _write_workspace_files(workspace: Path) -> None:
    (workspace / "EMBODIED.md").write_text(
        "# Embodied\n\n- Supports wave_hand and high_stand.\n",
        encoding="utf-8",
    )
    save_environment_doc(
        workspace / "ENVIRONMENT.md",
        {
            "schema_version": "PhyAgentOS.environment.v1",
            "robots": {
                "g1_001": {
                    "connection_state": {"status": "connected"},
                    "robot_pose": {"frame": "map", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
                }
            },
            "scene_graph": {"nodes": [], "edges": []},
            "objects": {},
        },
    )
    (workspace / "LESSONS.md").write_text("# Lessons\n", encoding="utf-8")


def test_wave_hand_shortcut_dispatches_embodied_action(tmp_path: Path) -> None:
    _write_workspace_files(tmp_path)

    provider = MagicMock()
    provider.get_default_model.return_value = "test-model"
    provider.estimate_prompt_tokens.return_value = (0, "test-counter")
    provider.chat_with_retry = AsyncMock(return_value=_FakeResponse("VALID"))

    loop = AgentLoop(
        bus=MessageBus(),
        provider=provider,
        workspace=tmp_path,
        model="test-model",
    )

    result = asyncio.run(loop.process_direct("wave hand", session_key="cli:test"))

    assert "validated and dispatched" in result
    action_doc = (tmp_path / "ACTION.md").read_text(encoding="utf-8")
    assert '"action_type": "wave_hand"' in action_doc
    assert '"robot_id": "g1_001"' in action_doc
    assert provider.chat_with_retry.await_count == 1
