from __future__ import annotations

import json
from pathlib import Path

from hal.drivers import load_driver
from hal.hal_watchdog import _poll_once
from hal.simulation.scene_io import load_environment_doc, save_environment_doc

_FENCE_OPEN = "```json"
_FENCE_CLOSE = "```"


def _write_action(workspace: Path, payload: dict) -> Path:
    action_file = workspace / "ACTION.md"
    action_file.write_text(
        f"{_FENCE_OPEN}\n{json.dumps(payload, indent=2)}\n{_FENCE_CLOSE}\n",
        encoding="utf-8",
    )
    return action_file


def _write_environment(workspace: Path) -> Path:
    env_file = workspace / "ENVIRONMENT.md"
    save_environment_doc(
        env_file,
        {
            "schema_version": "PhyAgentOS.environment.v1",
            "scene_graph": {"nodes": [], "edges": []},
            "robots": {},
            "objects": {},
        },
    )
    return env_file


def _write_fake_unitree_sdk(tmp_path: Path) -> Path:
    sdk_root = tmp_path / "fake_unitree_sdk"
    files = {
        "unitree_sdk2py/__init__.py": "",
        "unitree_sdk2py/core/__init__.py": "",
        "unitree_sdk2py/g1/__init__.py": "",
        "unitree_sdk2py/g1/loco/__init__.py": "",
        "unitree_sdk2py/core/channel.py": """
calls = []

def ChannelFactoryInitialize(domain_id, interface):
    calls.append((domain_id, interface))
""",
        "unitree_sdk2py/g1/loco/g1_loco_client.py": """
class LocoClient:
    def __init__(self):
        self.calls = []
        self.timeout = None
    def SetTimeout(self, timeout):
        self.timeout = timeout
        self.calls.append(("SetTimeout", timeout))
    def Init(self):
        self.calls.append(("Init",))
    def Move(self, vx, vy, vyaw, continuous_move=False):
        self.calls.append(("Move", vx, vy, vyaw, continuous_move))
    def StopMove(self):
        self.calls.append(("StopMove",))
    def Damp(self):
        self.calls.append(("Damp",))
    def Squat2StandUp(self):
        self.calls.append(("Squat2StandUp",))
    def StandUp2Squat(self):
        self.calls.append(("StandUp2Squat",))
    def Lie2StandUp(self):
        self.calls.append(("Lie2StandUp",))
    def LowStand(self):
        self.calls.append(("LowStand",))
    def HighStand(self):
        self.calls.append(("HighStand",))
    def ZeroTorque(self):
        self.calls.append(("ZeroTorque",))
    def WaveHand(self, turn_flag=False):
        self.calls.append(("WaveHand", turn_flag))
    def ShakeHand(self, stage=-1):
        self.calls.append(("ShakeHand", stage))
""",
    }
    for relative_path, content in files.items():
        path = sdk_root / relative_path
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content.lstrip(), encoding="utf-8")
    return sdk_root


def test_g1_driver_primitive_motion_forward_calls_unitree_sdk(tmp_path: Path) -> None:
    sdk_root = _write_fake_unitree_sdk(tmp_path)
    with load_driver(
        "g1",
        unitree_sdk_path=str(sdk_root),
        network_interface="dummy0",
        command_dt_s=0.05,
        linear_speed_mps=0.4,
    ) as driver:
        assert driver.connect() is True
        result = driver.execute_action(
            "primitive_motion",
            {"robot_id": "g1_001", "primitive": "forward", "distance_m": 0.08},
        )

        calls = driver._loco_client.calls

    assert "Moved forward" in result
    assert ("Init",) in calls
    assert any(call[0] == "Move" and call[1] > 0.0 for call in calls)
    assert ("StopMove",) in calls


def test_g1_driver_stop_and_pose_update(tmp_path: Path) -> None:
    sdk_root = _write_fake_unitree_sdk(tmp_path)
    env_file = _write_environment(tmp_path)
    action_file = _write_action(
        tmp_path,
        {
            "action_type": "primitive_motion",
            "parameters": {"robot_id": "g1_001", "primitive": "turn_right", "angle_deg": 90},
            "status": "pending",
        },
    )

    with load_driver(
        "g1",
        unitree_sdk_path=str(sdk_root),
        network_interface="dummy0",
        command_dt_s=0.05,
        angular_speed_rps=40.0,
    ) as driver:
        driver.connect()
        _poll_once(driver, action_file, env_file)
        stop_result = driver.execute_action("stop", {"robot_id": "g1_001"})
        calls = list(driver._loco_client.calls)

    updated = load_environment_doc(env_file)
    yaw = updated["robots"]["g1_001"]["robot_pose"]["yaw"]
    assert stop_result == "Motion stopped."
    assert yaw < -1.0
    assert calls.count(("StopMove",)) >= 2


def test_g1_driver_posture_actions_call_expected_sdk_methods(tmp_path: Path) -> None:
    sdk_root = _write_fake_unitree_sdk(tmp_path)
    with load_driver(
        "g1",
        unitree_sdk_path=str(sdk_root),
        network_interface="dummy0",
        command_dt_s=0.05,
    ) as driver:
        driver.connect()
        driver.execute_action("wave_hand", {"robot_id": "g1_001", "turn_around": True})
        driver.execute_action("high_stand", {"robot_id": "g1_001"})
        driver.execute_action("shake_hand", {"robot_id": "g1_001", "stage": 1})
        calls = list(driver._loco_client.calls)

    assert ("WaveHand", True) in calls
    assert ("HighStand",) in calls
    assert ("ShakeHand", 1) in calls
