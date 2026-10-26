
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")  # 在无图形界面的服务器上保存图片
import matplotlib.pyplot as plt


def plot_trajectory(name):
    # 1. 读取 C++ 生成的 CSV
    csv_path = Path(f"results/day011_{name}.csv")

    x = []
    y = []

    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            x.append(float(row["x_m"]))
            y.append(float(row["y_m"]))

    # 2. 绘制二维轨迹
    fig, ax = plt.subplots(figsize=(7, 6))

    ax.plot(x, y, label="Vehicle trajectory")
    ax.scatter(x[0], y[0],
               color="green", s=80, label="Start")
    ax.scatter(x[-1], y[-1],
               color="red", s=80, label="End")

    # 3. 设置坐标轴和图例
    ax.set_xlabel("X position (m)")
    ax.set_ylabel("Y position (m)")
    ax.set_title(f"Vehicle Trajectory: {name}")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True)
    ax.legend()

    # 4. 保存图片
    output = Path(f"results/day011_{name}.png")
    fig.savefig(output, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"Saved: {output}")


def main():
    plot_trajectory("straight")
    plot_trajectory("circle")
    plot_trajectory("s_curve")


if __name__ == "__main__":
    main()
