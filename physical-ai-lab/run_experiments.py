"""Run the four requested experiments plus an uncontrolled baseline."""

import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from main import run_experiment, save_results

BASE = Path(__file__).resolve().parent
CASES = [
    ("A_target45", 45, 15, 3),
    ("B_target90", 90, 15, 3),
    ("C_kp5", 45, 5, 3),
    ("D_kd1", 45, 15, 1),
    ("E_no_control", 45, 0, 0),
]


def main():
    summaries = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    for name, target, kp, kd in CASES:
        rows, metrics = run_experiment(target, kp, kd)
        save_results(rows, metrics, BASE / "results" / name)
        summaries.append({
            "experiment": name, "target_deg": target, "kp": kp, "kd": kd,
            "final_angle_deg": metrics["final_angle_deg"],
            "final_error_deg": metrics["final_error_deg"],
            "max_tail_error_deg": metrics["max_tail_0_5s_error_deg"],
            "settling_1deg_s": metrics["settling_time_1deg_s_within_horizon"],
            "overshoot_deg": metrics["overshoot_deg"],
            "saturation_fraction": metrics["saturation_fraction"],
            "passed": metrics["passed"],
        })
        times = [0] + [row["time_end_s"] for row in rows]
        angles = [0] + [row["angle_after_deg"] for row in rows]
        axes[0].plot(times, angles, label=name)
        axes[1].plot(times, [target - angle for angle in angles], label=name)
        print(f"{name}: final={metrics['final_angle_deg']:.6f} deg, "
              f"tail_error={metrics['max_tail_0_5s_error_deg']:.6f} deg, "
              f"passed={metrics['passed']}")
    for axis, label in zip(axes, ("Joint angle (deg)", "Target - angle (deg)")):
        axis.set_xlabel("Simulation time (s)")
        axis.set_ylabel(label)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8)
    axes[1].axhspan(-1, 1, color="green", alpha=0.12)
    fig.suptitle("Real MuJoCo runs: A/B/C/D and zero-input baseline")
    fig.tight_layout()
    fig.savefig(BASE / "results" / "comparison.png", dpi=150)
    plt.close(fig)
    with (BASE / "results" / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
        writer.writeheader()
        writer.writerows(summaries)


if __name__ == "__main__":
    main()
