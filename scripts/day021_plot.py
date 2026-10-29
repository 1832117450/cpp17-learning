import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path("results")

# 读取 CSV
def read(name):
    filename = root / f"day021_{name}.csv"

    with filename.open(newline="") as f:
        rows = list(csv.DictReader(f))

    return {
        k: [float(r[k]) for r in rows]
        for k in rows[0]
    }


normal = read("normal")
difficult = read("difficult")
adjusted = read("adjusted")

# =====================================
# 图 1：困难场景轨迹对比
# =====================================
plt.figure(figsize=(11, 5))

plt.plot(
    difficult["ref_x"],
    difficult["ref_y"],
    "k--",
    label="Reference",
    linewidth=1.4
)

plt.plot(
    difficult["x"],
    difficult["y"],
    label="Difficult: lookahead 15 m"
)

plt.plot(
    adjusted["x"],
    adjusted["y"],
    label="Adjusted: lookahead 4 m"
)

plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Day 021: Trajectory comparison at 16 m/s")
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()

plt.savefig(
    root / "day021_trajectory.png",
    dpi=180
)
plt.close()

# =====================================
# 图 2：跟踪误差对比
# =====================================
plt.figure(figsize=(10, 5))

plt.plot(
    difficult["t"],
    difficult["error"],
    label="Difficult: 15 m"
)

plt.plot(
    adjusted["t"],
    adjusted["error"],
    label="Adjusted: 4 m"
)

plt.axvline(
    4.0,
    linestyle=":",
    color="gray",
    label="After 4 s"
)

plt.xlabel("Time (s)")
plt.ylabel("Nearest-point error (m)")
plt.title("Day 021: Tracking error comparison")
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()

plt.savefig(
    root / "day021_error.png",
    dpi=180
)
plt.close()

# =====================================
# 图 3：转向响应对比
# =====================================
plt.figure(figsize=(10, 5))

plt.plot(
    difficult["t"],
    difficult["delta"],
    label="Difficult: 15 m"
)

plt.plot(
    adjusted["t"],
    adjusted["delta"],
    label="Adjusted: 4 m"
)

plt.xlabel("Time (s)")
plt.ylabel("Steering (rad)")
plt.title("Day 021: Steering response")
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()

plt.savefig(
    root / "day021_steering.png",
    dpi=180
)
plt.close()

for name in ("trajectory", "error", "steering"):
    print(
        "Saved:",
        root / f"day021_{name}.png"
    )
