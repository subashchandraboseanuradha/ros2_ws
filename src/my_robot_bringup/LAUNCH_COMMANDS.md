# My Robot Launch Commands

This document contains all the necessary commands to launch your robot system with Gazebo simulation, navigation, and teleoperation.

## Prerequisites

Source your workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Launch Commands

### 1. Launch Gazebo with Robot Simulation

```bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py use_sim_time:=true world:=/home/subash/ros2_ws/src/my_robot_description/world/house.world
```

**What it does:**

- Starts Gazebo simulator
- Loads your robot model
- Enables sim_time for synchronized simulation
- Loads the house world

---

### 2. Launch Twist Mux (Velocity Multiplexer)

```bash
ros2 run twist_mux twist_mux --ros-args --params-file /home/subash/ros2_ws/src/my_robot_bringup/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel
```

**What it does:**

- Manages multiple velocity command sources
- Routes commands to the robot's cmd_vel topic
- Uses configuration from twist_mux.yaml

---

### 3. Launch Joystick Teleoperation

```bash
ros2 launch my_robot_bringup joystick.launch.py use_sim_time:=true
```

**What it does:**

- Enables joystick control for your robot
- Synchronizes with simulation time

---

### 4. Launch Nav2 Navigation Stack

```bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/subash/ros2_ws/src/maps/my_map.yaml
```

**What it does:**

- Starts Nav2 navigation framework
- Loads your pre-built map
- Enables autonomous navigation capabilities

---

### 5. Launch RViz with Nav2 Configuration

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

**What it does:**

- Opens RViz visualization tool
- Loads Nav2 default configuration
- Visualizes navigation goals, paths, and sensor data

---

## Complete Startup Sequence (All in One)

Run each of these commands in separate terminals:

**Terminal 1 - Gazebo Simulation:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.py use_sim_time:=true world:=/home/subash/ros2_ws/src/my_robot_description/world/house.world
```

**Terminal 2 - Twist Mux:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run twist_mux twist_mux --ros-args --params-file /home/subash/ros2_ws/src/my_robot_bringup/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel
```

**Terminal 3 - Joystick Control:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup joystick.launch.py use_sim_time:=true
```

**Terminal 4 - Nav2 Navigation:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/subash/ros2_ws/src/maps/my_map.yaml
```

**Terminal 5 - RViz Visualization:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

---

## Quick Reference

| Command   | Purpose                      |
| --------- | ---------------------------- |
| Gazebo    | Robot simulation environment |
| Twist Mux | Velocity command management  |
| Joystick  | Teleoperation control        |
| Nav2      | Autonomous navigation        |
| RViz      | Visualization and monitoring |

---

## Troubleshooting

- **Gazebo doesn't start:** Check if Gazebo is installed: `sudo apt install ros-humble-gazebo-ros-pkgs`
- **Map not found:** Verify the map file exists at `/home/subash/ros2_ws/src/maps/my_map.yaml`
- **sim_time not synchronized:** Ensure all nodes use the same `use_sim_time` parameter
- **Joystick not detected:** Connect your joystick and check with `ls /dev/input/js*`
