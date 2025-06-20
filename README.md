# Four Wheel Bot Simulation in ROS 2 & Gazebo

## Project Overview

This project simulates a 4-wheeled differential drive robot using **ROS 2 (Humble)** and **Gazebo**. The robot is equipped with a **LIDAR** and a **camera** sensor. It can be controlled via keyboard using the **teleop_twist_keyboard** package. 

---

## Workspace Structure

```
four_wheel_ws/
└── src/
    └── four_wheel_bot/
        ├── launch/
        │   └── view-robot.launch.py          
        ├── obstacle_stop/
        │   ├── __init__.py
        │   └── obstacle_stop_node.py         
        ├── urdf/
        │   └── four_wheel_bot.xacro          
        ├── worlds/
        │   └── wall_world.world              
        ├── package.xml
        └── setup.py
```

---

## 📦 Required Packages

Ensure the following packages are installed:
- `ros-humble-desktop`
- `gazebo_ros_pkgs`
- `xacro`
- `teleop_twist_keyboard`

You can install missing ones with:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-xacro ros-humble-teleop-twist-keyboard
```

---

## 🚀 How to Build

```bash
cd ~/four_wheel_bot_ws
colcon build
source install/setup.bash
```

---

## 🧪 How to Simulate the Robot in Gazebo

### Step 1: Launch the simulation

> 📌 Open a new terminal

```bash
cd ~/four_wheel_bot_ws
source install/setup.bash
ros2 launch four_wheel_bot gazebo.launch.py
```

This command:
- Spawns the robot in Gazebo
- Loads your custom world
- Initializes the LIDAR and camera plugins

---

## 🎮 How to Control the Robot with Teleop

### Step 2: Launch teleop

> 📌 Open a new terminal

```bash
cd ~/four_wheel_bot_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use your keyboard to drive the robot:
- `W/A/S/D` – Move forward/left/backward/right
- `Q/E` – Increase/decrease speed

Make sure `cmd_vel` is being published and robot is responding in the simulation.

---

## 📷 Sensor Topics

- LIDAR: `/scan`
- Camera image: `/image_raw`
- Camera info: `/camera_info`
- Velocity command: `/cmd_vel`
- Odometry: `/odom`

---

## 🛠️ Troubleshooting

- If robot floats or misaligns, check:
  - Wheel joint origins
  - Sensor mount offsets
  - TF tree using `rqt_tf_tree` or `ros2 run tf2_tools view_frames`

---

## 📌 Next Steps

- Add SLAM or Navigation2 stack
- Implement obstacle avoidance
- Visualize in RViz

---

## 📃 License

MIT License © 2025
