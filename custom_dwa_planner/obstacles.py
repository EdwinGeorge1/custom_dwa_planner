import math

def compute_obstacle_points_in_odom(scan, robot_x, robot_y, robot_yaw):
    points = []
    if scan is None:
        return points

    angle = scan.angle_min
    for r in scan.ranges:
        if r is None or math.isinf(r) or math.isnan(r):
            angle += scan.angle_increment
            continue
        lx = r * math.cos(angle)
        ly = r * math.sin(angle)
        ox = robot_x + (lx * math.cos(robot_yaw) - ly * math.sin(robot_yaw))
        oy = robot_y + (lx * math.sin(robot_yaw) + ly * math.cos(robot_yaw))
        points.append((ox, oy))
        angle += scan.angle_increment
    return points
