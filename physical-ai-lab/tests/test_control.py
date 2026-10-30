"""Run with: python -m pytest -q. Tests use real MuJoCo dynamics."""

import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location("arm_main", Path(__file__).parents[1] / "main.py")
arm = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(arm)


def test_model_compiles_and_action_requires_physics_step():
    model, data = arm.load_model()
    assert (model.nq, model.nv, model.nu) == (1, 1, 1)
    assert model.opt.timestep == pytest.approx(0.002)
    before = data.qpos.copy()
    data.ctrl[0] = 1.0
    np.testing.assert_array_equal(data.qpos, before)
    mujoco.mj_step(model, data)
    assert data.time == pytest.approx(0.002)
    assert abs(data.qvel[0]) > 0
    assert abs(data.qpos[0]) > 0


@pytest.mark.parametrize("target", [45.0, 90.0, -45.0])
def test_control_converges_and_respects_torque_limit(target):
    rows, metrics = arm.run_experiment(target_deg=target)
    assert metrics["completed"] and metrics["steps"] == 1500
    assert metrics["max_tail_0_5s_error_deg"] < 1.0
    assert metrics["passed"]
    assert all(abs(row["torque_applied_nm"]) <= 10 for row in rows)
    assert rows[0]["angle_before_deg"] == 0
    assert rows[0]["time_end_s"] == pytest.approx(0.002)
    assert rows[-1]["angle_after_deg"] == pytest.approx(target, abs=1)


def test_repeatability_and_zero_input_baseline():
    rows_a, _ = arm.run_experiment()
    rows_b, _ = arm.run_experiment()
    assert rows_a == rows_b
    rows, metrics = arm.run_experiment(kp=0, kd=0)
    assert not metrics["passed"]
    assert all(row["angle_after_deg"] == 0 for row in rows)


@pytest.mark.parametrize("changes", [
    {"target_deg": 180}, {"target_deg": float("nan")}, {"kp": -1},
    {"kd": -1}, {"duration": 0}, {"duration": 0.001},
])
def test_invalid_experiment_rejected(changes):
    with pytest.raises(ValueError):
        arm.run_experiment(**changes)


def test_removing_joint_also_requires_removing_its_motor():
    root = ET.parse(arm.MODEL_PATH).getroot()
    body = root.find("worldbody/body")
    body.remove(body.find("joint"))
    with pytest.raises(ValueError):
        mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    root.remove(root.find("actuator"))
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
    assert model.nq == 0 and model.nu == 0


def test_result_files_have_aligned_time_and_observed_final_state(tmp_path):
    import csv
    import json

    rows, metrics = arm.run_experiment(duration=0.1)
    arm.save_results(rows, metrics, tmp_path)
    with (tmp_path / "joint_log.csv").open(encoding="utf-8") as stream:
        recorded = list(csv.DictReader(stream))
    assert len(recorded) == 50
    assert float(recorded[-1]["angle_after_deg"]) == metrics["final_angle_deg"]
    assert float(recorded[-1]["time_end_s"]) == pytest.approx(0.1)
    assert json.loads((tmp_path / "metrics.json").read_text())["steps"] == 50
    assert (tmp_path / "joint_angle.png").read_bytes().startswith(b"\x89PNG")
