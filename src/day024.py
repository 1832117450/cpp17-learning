
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

# 路径配置
project = Path(__file__).resolve().parents[1]
reference_repo = Path.home() / "projects" / "PythonRobotics"
sys.path.insert(0, str(reference_repo))

from PathTracking.lqr_steer_control import lqr_steer_control as lqr
from PathPlanning.CubicSpline import cubic_spline_planner

output = project / "results"
output.mkdir(exist_ok=True)

# 关闭实时动画
lqr.show_animation = False

# 与参考代码相同的道路
ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]

cx, cy, cyaw, ck, _ = \
    cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1
    )

goal = [ax[-1], ay[-1]]
speed = 10.0 / 3.6
sp = lqr.calc_speed_profile(cx, cy, cyaw, speed)


def run_experiment(label, q_lateral):
    # 只改变横向位置误差的权重
    lqr.Q = np.eye(4)
    lqr.Q[0, 0] = q_lateral
    lqr.R = np.eye(1)

    t, x, y, yaw, v = lqr.closed_loop_prediction(
        cx, cy, cyaw, ck, sp, goal
    )

    # 保存轨迹 CSV
    data = np.column_stack([t, x, y, yaw, v])

    np.savetxt(
        output / f"day024_{label}.csv",
        data,
        delimiter=",",
        header="time_s,x_m,y_m,yaw_rad,v_mps",
        comments=""
    )

    # 最近参考采样点距离 RMSE
    tree = cKDTree(np.column_stack([cx, cy]))
    distances, _ = tree.query(np.column_stack([x, y]))
    rmse = np.sqrt(np.mean(distances ** 2))

    print(f"{label}: Q[0,0]={q_lateral}, "
          f"RMSE={rmse:.4f} m, steps={len(t)}")

    # 保存单组轨迹图
    plt.figure(figsize=(8, 6))
    plt.plot(cx, cy, "k--", label="Reference")
    plt.plot(x, y, label=label)
    plt.axis("equal")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(output / f"day024_{label}.png", dpi=160)
    plt.close()

    return x, y


# 第一组：原始权重
x1, y1 = run_experiment("baseline", 1.0)

# 第二组：提高横向误差权重
x2, y2 = run_experiment("q5", 5.0)

# 绘制两组对比图
plt.figure(figsize=(8, 6))
plt.plot(cx, cy, "k--", label="Reference")
plt.plot(x1, y1, label="Baseline Q=I")
plt.plot(x2, y2, label="Modified Q[0,0]=5")
plt.axis("equal")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(output / "day024_compare.png", dpi=160)
plt.close()

print("Figures and CSV files saved to results/")
