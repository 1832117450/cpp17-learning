
import pandas as pd
import matplotlib.pyplot as plt

# P 和 PI 控制实验数据
p = pd.read_csv("results/day019_p.csv")
pi = pd.read_csv("results/day019_pi.csv")

# 绘制速度响应
plt.figure(figsize=(10, 6))

plt.plot(
    p["time"], p["target_speed"],
    "k--", label="Target Speed"
)

plt.plot(
    p["time"], p["actual_speed"],
    label="P Controller"
)

plt.plot(
    pi["time"], pi["actual_speed"],
    label="PI Controller"
)

plt.axhline(y=10, color="gray",
            linestyle=":", alpha=0.5)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("P vs PI Speed Control")
plt.legend()
plt.grid(True)

plt.savefig("results/day019_p_pi_compare.png", dpi=150)
plt.close()

# 绘制控制指令
plt.figure(figsize=(10, 6))

plt.plot(p["time"], p["control"], label="P Control")
plt.plot(pi["time"], pi["control"], label="PI Control")

plt.xlabel("Time (s)")
plt.ylabel("Acceleration Command (m/s^2)")
plt.title("P vs PI Control Input")
plt.legend()
plt.grid(True)

plt.savefig("results/day019_p_pi_control.png", dpi=150)
plt.close()

print("P vs PI comparison plots saved.")
