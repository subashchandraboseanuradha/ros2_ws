#!/bin/bash

# My Robot Complete Launch Script
# This script launches all necessary components for the robot system

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Source workspace
echo -e "${YELLOW}Sourcing ROS 2 workspace...${NC}"
source ~/ros2_ws/install/setup.bash

# Function to launch in background with logging
launch_in_terminal() {
    local name=$1
    local command=$2
    
    echo -e "${GREEN}Starting $name...${NC}"
    gnome-terminal -- bash -c "source ~/ros2_ws/install/setup.bash; $command; exec bash"
}

# Check if we can open terminals
if ! command -v gnome-terminal &> /dev/null; then
    echo -e "${RED}gnome-terminal not found. Please install it or run commands manually.${NC}"
    echo -e "${YELLOW}Use the LAUNCH_COMMANDS.md file for manual setup.${NC}"
    exit 1
fi

echo -e "${YELLOW}Starting My Robot System...${NC}"
echo ""

# Terminal 1: Gazebo Simulation
launch_in_terminal "Gazebo Simulation" \
    "ros2 launch my_robot_bringup my_robot_gazebo.launch.py use_sim_time:=true world:=/home/subash/ros2_ws/src/my_robot_description/world/house.world"

sleep 2

# Terminal 2: Twist Mux
launch_in_terminal "Twist Mux" \
    "ros2 run twist_mux twist_mux --ros-args --params-file /home/subash/ros2_ws/src/my_robot_bringup/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel"

sleep 2

# Terminal 3: Joystick Control
launch_in_terminal "Joystick Control" \
    "ros2 launch my_robot_bringup joystick.launch.py use_sim_time:=true"

sleep 2

# Terminal 4: Nav2 Navigation
launch_in_terminal "Nav2 Navigation" \
    "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/subash/ros2_ws/src/maps/my_map.yaml"

sleep 2

# Terminal 5: RViz Visualization
launch_in_terminal "RViz Visualization" \
    "ros2 run rviz2 rviz2 -d \$(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz"

echo -e "${GREEN}All systems launched!${NC}"
echo -e "${YELLOW}Check the opened terminals for status messages.${NC}"

