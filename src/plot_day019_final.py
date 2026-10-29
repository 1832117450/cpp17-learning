
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# 读取实验数据
datasets = {
    "P": pd.read_csv("results/day019_p.csv"),
    "PI": pd.read_csv("results/day019_pi.csv"),
    "PID": pd.read_csv("results/day019_pid.csv")
}

# 1. 绘制速度曲线
plt.figure(figsize=(10, 6))

for name, data in datasets.items():
    plt.plot(
        data["time"],
        data["actual_speed"],
        label=name
    )

plt.plot(
    datasets["P"]["time"],
    datasets["P"]["target_speed"],
    "k--",
    label="Target"
)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("P vs PI vs PID: Speed Tracking")
plt.legend()
plt.grid(True)
plt.savefig("results/day019_final_speed.png", dpi=150)
plt.close()

# 2. 绘制控制量曲线
plt.figure(figsize=(10, 6))

for name, data in datasets.items():
    plt.plot(
        data["time"],
        data["control"],
        label=name
    )

plt.xlabel("Time (s)")
plt.ylabel("Acceleration Command (m/s^2)")
plt.title("P vs PI vs PID: Control Input")
plt.legend()
plt.grid(True)
plt.savefig("results/day019_final_control.png", dpi=150)
plt.close()

# 3. 计算评估指标
print("\n=== Controller Comparison ===")

for name, data in datasets.items():
    # 仅统计目标速度阶跃后的区间
    data = data[data["time"] >= 1.0]

    t = data["time"].to_numpy()
    v = data["actual_speed"].to_numpy()
    target = data["target_speed"].to_numpy()

    error = target - v

    final_error = abs(error[-1])
    max_overshoot = max(0.0, np.max(v - target))

    # 积分绝对误差 IAE
    abs_error = np.abs(error)
    iae = np.sum(
        0.5 * (abs_error[:-1] + abs_error[1:])
        * np.diff(t)
    )

    print(f"\n{name} Controller")
    print(f"Final Error: {final_error:.3f} m/s")
    print(f"Max Overshoot: {max_overshoot:.3f} m/s")
    print(f"IAE: {iae:.3f} m")

print("\nPlots saved successfully.")
