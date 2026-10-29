
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS = Path("results")

LOOKAHEADS = [
    ("2p5", "Ld=2.5m"),
    ("5p0", "Ld=5.0m"),
    ("10p0", "Ld=10.0m"),
]

SCENARIOS = [
    ("straight", "Straight road, zero initial offset"),
    ("circle", "20m-radius circular road"),
    ("offset", "Straight road, initial offset=2m"),
]

fig, axes = plt.subplots(
    3, 2, figsize=(13, 12), constrained_layout=True
)

for row, (scenario, title) in enumerate(SCENARIOS):

    ax_path, ax_error = axes[row]

    # 绘制参考路径
    if scenario == "circle":
        theta = [
            2 * math.pi * i / 360
            for i in range(361)
        ]

        ax_path.plot(
            [20 * math.cos(t) for t in theta],
            [20 * math.sin(t) for t in theta],
            "k--",
            label="Reference"
        )
    else:
        ax_path.plot(
            [0, 160], [0, 0],
            "k--",
            label="Reference"
        )

    # 加载不同前视距离的实验结果
    for tag, label in LOOKAHEADS:

        filepath = (
            RESULTS / f"day017_{scenario}_ld{tag}.csv"
        )

        with filepath.open(
            newline="", encoding="utf-8"
        ) as f:
            samples = list(csv.DictReader(f))

        times = [float(s["time"]) for s in samples]
        xs = [float(s["x"]) for s in samples]
        ys = [float(s["y"]) for s in samples]
        errors = [float(s["error_m"]) for s in samples]

        # 左图：实际行驶轨迹
        ax_path.plot(xs, ys, label=label)

        # 右图：跟踪误差随时间变化
        ax_error.plot(times, errors, label=label)

    ax_path.set_title(title + ": trajectory")
    ax_path.set_xlabel("x (m)")
    ax_path.set_ylabel("y (m)")
    ax_path.grid(alpha=0.3)
    ax_path.legend()

    if scenario == "circle":
        ax_path.set_aspect("equal", adjustable="box")
    else:
        ax_path.set_ylim(-3, 3)

    ax_error.set_title(title + ": lateral error")
    ax_error.set_xlabel("Time (s)")
    ax_error.set_ylabel("Absolute lateral error (m)")
    ax_error.grid(alpha=0.3)
    ax_error.legend()

# 保存最终对比图
out = RESULTS / "day017_lookahead_comparison.png"

fig.savefig(out, dpi=160)
plt.close(fig)

print(f"Saved: {out}")
