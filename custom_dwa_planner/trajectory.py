import math

def predict_trajectory(x, y, yaw, v, w, dt, predict_time):
    traj, t = [], 0.0
    x_sim, y_sim, yaw_sim = x, y, yaw
    while t <= predict_time + 1e-6:
        x_sim += v * math.cos(yaw_sim) * dt
        y_sim += v * math.sin(yaw_sim) * dt
        yaw_sim += w * dt
        traj.append((x_sim, y_sim))
        t += dt
    return traj


def evaluate_trajectory(traj, v, goal_x, goal_y, obs_points, robot_radius,
                        weight_heading, weight_clearance, weight_velocity):
    if not traj:
        return float('-inf')

    final_x, final_y = traj[-1]
    heading_cost = -math.hypot(goal_x - final_x, goal_y - final_y)

    clearance = float('inf')
    for (px, py) in traj:
        for (ox, oy) in obs_points:
            d = math.hypot(px - ox, py - oy)
            clearance = min(clearance, d)
            if d <= robot_radius:
                return float('-inf')

    clearance_cost = clearance if clearance != float('inf') else 10.0
    velocity_cost = v

    return (weight_heading * heading_cost +
            weight_clearance * clearance_cost +
            weight_velocity * velocity_cost)
