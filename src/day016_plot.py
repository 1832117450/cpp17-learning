
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# =====================================
# 1. 设置文件路径
# =====================================
project_dir = Path(__file__).resolve().parent.parent
results_dir = project_dir / "results"

csv_path = results_dir / "day016_trajectory.csv"

# =====================================
# 2. 读取车辆轨迹
# =====================================
time = []
x = []
y = []
error = []
steering = []

with open(csv_path, "r", newline="") as file:
    reader = csv.DictReader(file)

    for row in reader:
        time.append(float(row["time"]))
        x.append(float(row["x"]))
        y.append(float(row["y"]))
        error.append(float(row["radial_error"]))
        steering.append(float(row["steering_deg"]))

# =====================================
# 3. 生成参考圆
# =====================================
radius = 20.0
angles = [2 * math.pi * i / 360 for i in range(361)]

ref_x = [radius * math.cos(a) for a in angles]
ref_y = [radius * math.sin(a) for a in angles]

# =====================================
# 4. 绘制路径跟踪结果
# =====================================
plt.figure(figsize=(8, 8))

plt.plot(
    ref_x, ref_y,
    "k--",
    label="Reference Path",
    linewidth=2
)

plt.plot(
    x, y,
    "b-",
    label="Vehicle Trajectory",
    linewidth=2
)

plt.scatter(
    x[0], y[0],
    color="red",
    s=80,
    label="Start",
    zorder=5
)

plt.scatter(
    x[-1], y[-1],
    color="green",
    s=80,
    label="End",
    zorder=5
)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Pure Pursuit Circular Path Tracking")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    results_dir / "day016_tracking.png",
    dpi=200
)
plt.close()

# =====================================
# 5. 绘制误差和转向角
# =====================================
plt.figure(figsize=(10, 5))
plt.plot(time, error, label="Radial Error")

plt.axhline(0, color="black", linestyle="--")

plt.xlabel("Time (s)")
plt.ylabel("Radial Error (m)")
plt.title("Pure Pursuit Tracking Error")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    results_dir / "day016_error.png",
    dpi=200
)
plt.close()

plt.figure(figsize=(10, 5))
plt.plot(time, steering, label="Steering Angle")

plt.xlabel("Time (s)")
plt.ylabel("Steering Angle (deg)")
plt.title("Pure Pursuit Steering Control")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig(
    results_dir / "day016_steering.png",
    dpi=200
)
plt.close()

# =====================================
# 6. 计算稳定阶段误差
# =====================================
steady_errors = [
    e for t, e in zip(time, error) if t >= 10.0
]

steady_rmse = math.sqrt(
    sum(e * e for e in steady_errors)
    / len(steady_errors)
)

print("Visualization completed.")
print(f"Steady-state RMSE (t >= 10s): {steady_rmse:.4f} m")
print("Saved: results/day016_tracking.png")
print("Saved: results/day016_error.png")
print("Saved: results/day016_steering.png")
