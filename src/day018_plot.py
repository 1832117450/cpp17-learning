
import csv
import math
import matplotlib.pyplot as plt

with open("results/day018_trajectory.csv") as f:
    data = list(csv.DictReader(f))

t = [float(r["time"]) for r in data]
x = [float(r["x"]) for r in data]
y = [float(r["y"]) for r in data]
yaw = [float(r["yaw"]) for r in data]
error = [
    float(r["cross_track_error"]) for r in data
]

# Stanley 使用前轴中心进行误差计算
front_x = [
    x[i] + 2.7 * math.cos(yaw[i])
    for i in range(len(x))
]
front_y = [
    y[i] + 2.7 * math.sin(yaw[i])
    for i in range(len(y))
]

# S 形参考路径
ref_x = [i * 0.2 for i in range(501)]
ref_y = [
    2.0 * math.sin(0.12 * xx)
    for xx in ref_x
]

# 图1：参考路径与实际跟踪路径
plt.figure(figsize=(10, 4.5))
plt.plot(ref_x, ref_y, "k--",
         label="Reference path")
plt.plot(front_x, front_y, "b",
         label="Stanley (front axle)")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Stanley Path Tracking")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.xlim(0, 80)
plt.savefig("results/day018_path.png", dpi=160)
plt.close()

# 图2：横向误差
plt.figure(figsize=(10, 4))
plt.plot(t, error)
plt.axhline(0, color="black",
            linestyle="--", linewidth=1)
plt.xlabel("Time (s)")
plt.ylabel("Cross-track error (m)")
plt.title("Stanley Tracking Error")
plt.grid(True)
plt.savefig("results/day018_error.png", dpi=160)
plt.close()

print("Plots saved successfully.")
