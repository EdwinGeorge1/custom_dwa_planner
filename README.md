# Custom DWA Local Planner (ROS2 Humble)

This package implements a **Dynamic Window Approach (DWA) local planner** from scratch in Python for a TurtleBot3 robot in Gazebo.  
It replaces the default `nav2_dwb_controller` with a lightweight, fully custom implementation.

---

## What is DWA?

The **Dynamic Window Approach (DWA)** is a local planner for mobile robots that selects the best velocity command $(v, \omega)$ at each time step based on trajectory simulation, obstacle avoidance, and goal direction.  
It does not compute a global path but continuously evaluates feasible motion commands for safe and efficient navigation.

---

### 🔹 Algorithm Steps

1. **Velocity Sampling**  
   Generate candidate linear velocities $v$ and angular velocities $\omega$ within the robot's dynamic constraints.

2. **Trajectory Prediction**  
   For each pair $(v, \omega)$, simulate short-horizon motion using:

$$
\begin{aligned}
x_{t+1} &= x_t + v \cdot \cos(\theta) \cdot \Delta t \\
y_{t+1} &= y_t + v \cdot \sin(\theta) \cdot \Delta t \\
\theta_{t+1} &= \theta_t + \omega \cdot \Delta t
\end{aligned}
$$

3. **Trajectory Evaluation**  
   Score each trajectory by:
   - **Heading cost:** How well does the trajectory point to the goal?  
   - **Clearance cost:** Minimum distance to nearest obstacle.  
   - **Velocity cost:** Rewards faster motion when safe.

   Full score formula:

$$
G(v, \omega) = w_{\text{heading}} \cdot f_{\text{heading}} + 
w_{\text{clearance}} \cdot f_{\text{clearance}} + 
w_{\text{velocity}} \cdot f_{\text{velocity}}
$$


4. **Command Selection**  
   Choose the $(v, \omega)$ with the highest score.  
   If all candidate trajectories are in collision, the robot rotates in place as a recovery.

---

## Features

- Complete **DWA local planner** written in Python (`rclpy`)
- Visualizes:
  - **White lines:** all candidate trajectories
  - **Green line:** the selected best trajectory
- Publishes velocity commands to `/cmd_vel` for TurtleBot3
- Designed for **Gazebo + RViz2** simulation environments
- Clear and lightweight codebase for learning and research

---

## Installation & Build

Clone into your ROS2 workspace `src` folder:

```bash
cd ~/ros2_ws/src
git clone <your_repo_link> custom_dwa_planner
cd ~/ros2_ws
colcon build --packages-select custom_dwa_planner
source install/setup.bash
