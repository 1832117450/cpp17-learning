
import csv
import matplotlib.pyplot as plt

time = []
target_speed = []
actual_speed = []
control = []

# 读取 C++ 仿真结果
with open("results/day019_p.csv", "r") as f:
    reader = csv.DictReader(f)

    for row in reader:
        time.append(float(row["time"]))
        target_speed.append(float(row["target_speed"]))
        actual_speed.append(float(row["actual_speed"]))
        control.append(float(row["control"]))

# 图1：速度跟踪曲线
plt.figure(figsize=(9, 5))

plt.plot(time, target_speed,
         "r--", label="Target Speed")

plt.plot(time, actual_speed,
         "b-", label="Actual Speed")

plt.xlabel("Time (s)")
plt.ylabel("Speed (m/s)")
plt.title("P Controller - Speed Tracking")
plt.legend()
plt.grid(True)

plt.savefig("results/day019_speed.png", dpi=150)
plt.close()

# 图2：加速度控制指令
plt.figure(figsize=(9, 5))

plt.plot(time, control,
         "g-", label="Acceleration Command")

plt.xlabel("Time (s)")
plt.ylabel("Acceleration (m/s^2)")
plt.title("P Controller - Control Input")
plt.legend()
plt.grid(True)

plt.savefig("results/day019_control.png", dpi=150)
plt.close()

print("Plots saved successfully.")
