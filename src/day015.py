import math


def evaluate_tracking(states, ref_y=0.0, ref_yaw_deg=0.0):
    """计算直线参考路径的跟踪误差。"""
    if not states:
        raise ValueError("states cannot be empty")

    lateral_errors = []

    for x, y, yaw_deg in states:
        # 横向误差：参考路径沿X轴正方向
        e_y = y - ref_y

        # 航向误差：先转弧度，再归一化
        angle = math.radians(yaw_deg - ref_yaw_deg)
        e_psi = math.atan2(math.sin(angle), math.cos(angle))

        lateral_errors.append(e_y)

        print(
            f"x={x:.1f}, y={y:.1f}, "
            f"e_y={e_y:.2f} m, "
            f"e_psi={math.degrees(e_psi):.2f} deg"
        )

    n = len(lateral_errors)

    rmse = math.sqrt(
        sum(e * e for e in lateral_errors) / n
    )

    max_error = max(abs(e) for e in lateral_errors)

    return rmse, max_error


if __name__ == "__main__":
    # 车辆状态：(x, y, 航向角[度])
    states = [
        (0.0, 0.0, 0.0),
        (1.0, 1.0, 5.0),
        (2.0, -2.0, -10.0),
        (3.0, 1.0, 3.0),
    ]

    rmse, max_error = evaluate_tracking(states)

    print(f"Lateral RMSE: {rmse:.3f} m")
    print(f"Max absolute error: {max_error:.3f} m")
