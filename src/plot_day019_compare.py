
import pandas as pd
import matplotlib.pyplot as plt

# 读取三组实验数据
data_02 = pd.read_csv("results/day019_p_kp02.csv")
data_05 = pd.read_csv("results/day019_p.csv")
data_10 = pd.read_csv("results/day019_p_kp10.csv")

plt.figure(figsize=(10, 6))

# 目标速度
plt.plot(
    data_05["time"],
    data_05["target_speed"],
    "k--",
    label="Target Speed"
)

# 三组实际速度
plt.plot(
    data_02["time"],
    data_02["actual_speed"],
    label="Kp = 0.2"
)

plt.plot(
    data_05["time"],
    data_05["actual_speed"],
    label="Kp = 0.5"
)

plt.plot(
    data_10["time"],
    data_10["actual_speed"],
    label="Kp = 1.0"
)

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("P Controller: Different Kp Values")
plt.legend()
plt.grid(True)

plt.savefig("results/day019_kp_compare.png", dpi=150)
plt.close()

print("Comparison plot saved successfully.")
