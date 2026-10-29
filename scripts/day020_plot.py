import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS = Path("results")

# ==============================
# 1. 读取 C++ 实验数据
# ==============================
def load_csv(filename):
    data = {}

    with open(filename, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)

        for row in reader:
            for key, value in row.items():
                data.setdefault(key, []).append(float(value))

    return data


pp = load_csv(RESULTS / "day020_pp_compare.csv")
st = load_csv(RESULTS / "day020_stanley.csv")


# ==============================
# 2. 保存图片
# ==============================
def save_plot(filename):
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        RESULTS / filename,
        dpi=180,
        bbox_inches="tight"
    )
    plt.close()

    print("Saved:", RESULTS / filename)


# ==============================
# 3. 轨迹跟踪对比
# ==============================
plt.figure(figsize=(11, 5))

plt.plot(
    pp["ref_x"],
    pp["ref_y"],
    "k--",
    label="Reference Path",
    linewidth=1.5
)

plt.plot(
    pp["x"],
    pp["y"],
    label="Pure Pursuit + PID",
    linewidth=1.5
)

plt.plot(
    st["x"],
    st["y"],
    label="Stanley + PID",
    linewidth=1.5
)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Day 020: Trajectory Tracking Comparison")

save_plot("day020_trajectory.png")


# ==============================
# 4. 跟踪误差对比
# ==============================
plt.figure(figsize=(10, 5))

plt.plot(
    pp["t"],
    pp["error"],
    label="Pure Pursuit",
    linewidth=1.5
)

plt.plot(
    st["t"],
    st["error"],
    label="Stanley",
    linewidth=1.5
)

plt.xlabel("Time (s)")
plt.ylabel("Tracking Error (m)")
plt.title("Day 020: Tracking Error Comparison")

save_plot("day020_tracking_error.png")


# ==============================
# 5. 转向控制对比
# ==============================
plt.figure(figsize=(10, 5))

plt.plot(
    pp["t"],
    pp["delta"],
    label="Pure Pursuit",
    linewidth=1.5
)

plt.plot(
    st["t"],
    st["delta"],
    label="Stanley",
    linewidth=1.5
)

plt.xlabel("Time (s)")
plt.ylabel("Steering Angle (rad)")
plt.title("Day 020: Steering Control Comparison")

save_plot("day020_steering.png")


# ==============================
# 6. 速度控制对比
# ==============================
plt.figure(figsize=(10, 5))

plt.plot(
    pp["t"],
    pp["v"],
    label="Pure Pursuit + PID"
)

plt.plot(
    st["t"],
    st["v"],
    "--",
    label="Stanley + PID"
)

plt.axhline(
    y=8.0,
    linestyle=":",
    color="black",
    label="Target Speed"
)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("Day 020: Speed Tracking Comparison")

save_plot("day020_speed.png")


print("\nAll Day 020 plots generated successfully.")
