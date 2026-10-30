"""Single-joint PD control through the real MuJoCo engine (no state teleporting)."""

import argparse
from contextlib import nullcontext
import csv
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import platform
import sys
import time

import mujoco
import numpy as np

BASE = Path(__file__).resolve().parent
MODEL_PATH = BASE / "models" / "single_joint.xml"


def load_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def validate(model, target_deg, kp, kd, duration):
    values = (target_deg, kp, kd, duration)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("All numeric arguments must be finite.")
    if kp < 0 or kd < 0 or duration <= 0:
        raise ValueError("kp/kd must be non-negative; duration must be positive.")
    target = math.radians(target_deg)
    lower, upper = model.joint("joint1").range
    if not lower <= target <= upper:
        raise ValueError("Target is outside the model joint range (radians: -3.14 .. 3.14).")
    if duration < model.opt.timestep:
        raise ValueError("Duration must be at least one physics timestep.")
    return target


def run_experiment(target_deg=45.0, kp=15.0, kd=3.0, duration=3.0, viewer=False):
    model, data = load_model()
    target = validate(model, target_deg, kp, kd, duration)
    lower, upper = model.actuator("joint_motor").ctrlrange
    motor_id = model.actuator("joint_motor").id
    initial_qpos = data.qpos.copy().tolist()
    initial_qvel = data.qvel.copy().tolist()
    requested_steps = int(math.ceil(duration / model.opt.timestep - 1e-10))
    rows = []
    context = nullcontext(None)
    if viewer:
        from mujoco import viewer as mj_viewer

        context = mj_viewer.launch_passive(model, data)
    with context as handle:
        if handle is not None:
            with handle.lock():
                handle.cam.lookat[:] = [0.25, 0, 0]
                handle.cam.distance = 1.6
                handle.cam.azimuth = 90
                handle.cam.elevation = -15
            handle.sync()
        for _ in range(requested_steps):
            if handle is not None and not handle.is_running():
                break
            wall_start = time.perf_counter()
            # Observation: use current simulated joint state, in radians and rad/s.
            q = float(data.joint("joint1").qpos[0])
            velocity = float(data.joint("joint1").qvel[0])
            error = target - q
            # Static angle target: desired velocity = 0. Action is torque, not angle.
            raw_torque = kp * error - kd * velocity
            torque = float(np.clip(raw_torque, lower, upper))
            start_time = float(data.time)
            data.ctrl[motor_id] = torque
            # Only MuJoCo advances qpos/qvel. The controller never assigns either.
            mujoco.mj_step(model, data)
            if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
                raise RuntimeError("Non-finite state detected; simulation aborted.")
            q_after = float(data.joint("joint1").qpos[0])
            rows.append({
                "time_start_s": start_time,
                "time_end_s": float(data.time),
                "target_deg": target_deg,
                "angle_before_deg": math.degrees(q),
                "velocity_before_rad_s": velocity,
                "error_before_deg": math.degrees(error),
                "torque_raw_nm": raw_torque,
                "torque_applied_nm": torque,
                "angle_after_deg": math.degrees(q_after),
                "velocity_after_rad_s": float(data.joint("joint1").qvel[0]),
                "error_after_deg": math.degrees(target - q_after),
            })
            if handle is not None:
                handle.sync()
                time.sleep(max(0, model.opt.timestep - (time.perf_counter() - wall_start)))
    if not rows:
        raise RuntimeError("No physics steps completed; no successful run to report.")
    times = np.array([row["time_end_s"] for row in rows])
    errors = np.array([row["error_after_deg"] for row in rows])
    angles = np.array([row["angle_after_deg"] for row in rows])
    outside = np.flatnonzero(np.abs(errors) > 1.0)
    settling = None
    if abs(errors[-1]) <= 1.0:
        index = int(outside[-1] + 1) if len(outside) else 0
        settling = float(times[index])
    direction = math.copysign(1.0, target_deg) if target_deg else 0.0
    overshoot = float(max(0, np.max(direction * (angles - target_deg))))
    saturated = sum(abs(r["torque_raw_nm"] - r["torque_applied_nm"]) > 1e-12 for r in rows)
    tail_error = float(np.max(np.abs(errors[times >= max(0, times[-1] - 0.5)])))
    complete = len(rows) == requested_steps
    metrics = {
        "target_deg": target_deg, "kp_nm_per_rad": kp, "kd_nm_s_per_rad": kd,
        "requested_duration_s": duration, "simulation_duration_s": float(data.time),
        "timestep_s": float(model.opt.timestep), "steps": len(rows),
        "completed": complete, "viewer_requested": viewer,
        "stop_reason": "duration_completed" if complete else "viewer_closed_early",
        "initial_qpos_rad": initial_qpos, "initial_qvel_rad_s": initial_qvel,
        "gravity_m_s2": model.opt.gravity.tolist(),
        "joint_damping_nm_s_per_rad": float(model.dof_damping[0]),
        "torque_limits_nm": [float(lower), float(upper)],
        "final_angle_deg": float(angles[-1]), "final_error_deg": float(errors[-1]),
        "final_velocity_rad_s": float(data.qvel[0]),
        "max_tail_0_5s_error_deg": tail_error,
        "rmse_deg": float(np.sqrt(np.mean(errors ** 2))),
        "overshoot_deg": overshoot,
        "settling_time_1deg_s_within_horizon": settling,
        "saturation_fraction": saturated / len(rows),
        "criterion": "complete AND max error in final 0.5s <= 1 deg (teaching criterion)",
        "passed": bool(complete and tail_error <= 1.0),
        "python": sys.version, "platform": platform.platform(),
        "packages": {name: importlib.metadata.version(name)
                     for name in ("mujoco", "numpy", "matplotlib")},
        "model_sha256": hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest(),
        "code_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    return rows, metrics


def save_results(rows, metrics, output):
    # Headless plots work without a desktop; --viewer explicitly opens the 3D GUI.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    with (output / "joint_log.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    times = [0.0] + [row["time_end_s"] for row in rows]
    angles = [math.degrees(metrics["initial_qpos_rad"][0])]
    angles += [row["angle_after_deg"] for row in rows]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].plot(times, angles, label="MuJoCo joint angle")
    axes[0].axhline(metrics["target_deg"], color="black", linestyle="--", label="Target")
    axes[0].set_ylabel("Angle (deg)")
    axes[0].legend()
    axes[1].plot(times, np.array(metrics["target_deg"]) - np.array(angles))
    axes[1].axhspan(-1, 1, color="green", alpha=0.1, label="Teaching tolerance: +/-1 deg")
    axes[1].set_ylabel("Error (deg)")
    axes[1].legend()
    axes[2].plot([r["time_start_s"] for r in rows],
                 [r["torque_applied_nm"] for r in rows])
    axes[2].set_ylabel("Input torque (Nm)")
    axes[2].set_xlabel("Simulation time (s)")
    for axis in axes:
        axis.grid(alpha=0.25)
    fig.suptitle(f"PD control: target={metrics['target_deg']:g} deg, "
                 f"Kp={metrics['kp_nm_per_rad']:g}, Kd={metrics['kd_nm_s_per_rad']:g}")
    fig.tight_layout()
    fig.savefig(output / "joint_angle.png", dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=float, default=45.0, help="Target angle in degrees")
    parser.add_argument("--kp", type=float, default=15.0)
    parser.add_argument("--kd", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--viewer", action="store_true")
    parser.add_argument("--output", type=Path, default=BASE / "results")
    args = parser.parse_args()
    try:
        rows, metrics = run_experiment(args.target, args.kp, args.kd, args.duration, args.viewer)
    except ValueError as error:
        parser.error(str(error))
    save_results(rows, metrics, args.output)
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    print(f"Saved CSV, PNG and metrics to: {args.output.resolve()}")


if __name__ == "__main__":
    main()
