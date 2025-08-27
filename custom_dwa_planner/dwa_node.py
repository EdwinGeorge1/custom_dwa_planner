#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import math
import numpy as np

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray

# Local imports
from custom_dwa_planner.trajectory import predict_trajectory, evaluate_trajectory
from custom_dwa_planner.obstacles import compute_obstacle_points_in_odom
from custom_dwa_planner.visualization import publish_markers
from custom_dwa_planner.utils import get_yaw


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_local_planner')

        # ----------------- Publishers -----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)

        # ----------------- Subscribers -----------------
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # ----------------- State -----------------
        self.goal_x = None
        self.goal_y = None
        self.current_pose = None
        self.scan_msg = None

        # ----------------- DWA Parameters -----------------
        self.max_speed = 0.3          # [m/s]
        self.min_speed = 0.0          # [m/s]
        self.max_yaw_rate = 1.0       # [rad/s]
        self.dt = 0.1                 # [s] step time
        self.predict_time = 2.0       # [s] forward simulation time
        self.robot_radius = 0.25      # [m] for collision checking

        # ----------------- Cost Weights -----------------
        self.weight_heading = 1.0
        self.weight_clearance = 1.2
        self.weight_velocity = 0.1

        self.get_logger().info("✅ Custom DWA Planner Started (waiting for /goal_pose)")

        # Main loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

    # ----------------- Callbacks -----------------
    def goal_callback(self, msg: PoseStamped):
        """Receive new navigation goal."""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.get_logger().info(f"🎯 New goal: ({self.goal_x:.2f}, {self.goal_y:.2f})")

    def odom_callback(self, msg: Odometry):
        """Update current robot pose from odometry."""
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg: LaserScan):
        """Update latest laser scan."""
        self.scan_msg = msg

    # ----------------- Main Control Loop -----------------
    def control_loop(self):
        """Runs at 10 Hz, generates and evaluates trajectories."""
        if self.current_pose is None or self.scan_msg is None or self.goal_x is None:
            return  # Wait until odom, scan, and goal are available

        # Current robot state
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        yaw = get_yaw(self.current_pose.orientation)

        # Distance to goal
        dx, dy = self.goal_x - x, self.goal_y - y
        goal_dist = math.hypot(dx, dy)

        # Stop if goal is reached
        if goal_dist < 0.15:
            self.cmd_pub.publish(Twist())  # Publish zero velocity
            self.get_logger().info("🎯 Goal reached! Stopping robot.")
            return

        # Velocity samples
        v_samples = np.linspace(self.min_speed, self.max_speed, 6)
        w_samples = np.linspace(-self.max_yaw_rate, self.max_yaw_rate, 13)

        # Initialize best trajectory
        best_score, best_u, best_traj = float('-inf'), [0.0, 0.0], []
        all_trajs = []

        # Compute obstacle points in odom frame
        obs_points = compute_obstacle_points_in_odom(self.scan_msg, x, y, yaw)

        # ----------------- Trajectory Evaluation -----------------
        for v in v_samples:
            for w in w_samples:
                traj = predict_trajectory(x, y, yaw, v, w, self.dt, self.predict_time)
                score = evaluate_trajectory(
                    traj, v, self.goal_x, self.goal_y,
                    obs_points, self.robot_radius,
                    self.weight_heading, self.weight_clearance, self.weight_velocity
                )
                all_trajs.append(traj)

                if score > best_score:
                    best_score, best_u, best_traj = score, [v, w], traj

        # If no valid trajectory found → rotate in place
        if best_score == float('-inf'):
            self.get_logger().warn("⚠️ All trajectories in collision, rotating in place")
            cmd = Twist()
            cmd.angular.z = 0.6
            self.cmd_pub.publish(cmd)
            publish_markers(self.marker_pub, all_trajs, [], self.get_clock())
            return

        # ----------------- Publish Best Command -----------------
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = best_u
        self.cmd_pub.publish(cmd)

        # ----------------- Visualization -----------------
        publish_markers(self.marker_pub, all_trajs, best_traj, self.get_clock())


# ----------------- Entry Point -----------------
def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
