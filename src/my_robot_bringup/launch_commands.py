#!/usr/bin/env python3

import subprocess
import time
import sys
from pathlib import Path

# Define colors for terminal output
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    END = '\033[0m'

def print_header(text):
    print(f"\n{Colors.BLUE}{'='*60}{Colors.END}")
    print(f"{Colors.BLUE}{text.center(60)}{Colors.END}")
    print(f"{Colors.BLUE}{'='*60}{Colors.END}\n")

def print_success(text):
    print(f"{Colors.GREEN}✓ {text}{Colors.END}")

def print_warning(text):
    print(f"{Colors.YELLOW}⚠ {text}{Colors.END}")

def print_error(text):
    print(f"{Colors.RED}✗ {text}{Colors.END}")

def verify_paths():
    """Verify that all required paths and files exist."""
    print_header("Verifying Required Paths")
    
    paths = {
        'Workspace': Path.home() / 'ros2_ws',
        'Map file': Path.home() / 'ros2_ws/src/maps/my_map.yaml',
        'Twist Mux config': Path.home() / 'ros2_ws/src/my_robot_bringup/config/twist_mux.yaml',
        'World file': Path.home() / 'ros2_ws/src/my_robot_description/world/house.world',
    }
    
    all_exist = True
    for name, path in paths.items():
        if path.exists():
            print_success(f"{name}: {path}")
        else:
            print_error(f"{name} NOT FOUND: {path}")
            all_exist = False
    
    return all_exist

def run_commands():
    """Run the launch commands sequentially."""
    print_header("My Robot System Launcher")
    
    # Verify paths first
    if not verify_paths():
        print_error("Some required files are missing!")
        print_warning("Please check the paths and ensure all files exist.")
        user_input = input(f"\n{Colors.YELLOW}Continue anyway? (y/n): {Colors.END}")
        if user_input.lower() != 'y':
            sys.exit(1)
    
    # Define commands
    commands = [
        {
            'name': 'Gazebo Simulation',
            'command': 'ros2 launch my_robot_bringup my_robot_gazebo.launch.py use_sim_time:=true world:=/home/subash/ros2_ws/src/my_robot_description/world/house.world',
            'description': 'Gazebo simulator with robot and house world'
        },
        {
            'name': 'Twist Mux',
            'command': 'ros2 run twist_mux twist_mux --ros-args --params-file /home/subash/ros2_ws/src/my_robot_bringup/config/twist_mux.yaml -r cmd_vel_out:=/cmd_vel',
            'description': 'Velocity command multiplexer'
        },
        {
            'name': 'Joystick Control',
            'command': 'ros2 launch my_robot_bringup joystick.launch.py use_sim_time:=true',
            'description': 'Joystick teleoperation interface'
        },
        {
            'name': 'Nav2 Navigation',
            'command': 'ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/home/subash/ros2_ws/src/maps/my_map.yaml',
            'description': 'Nav2 autonomous navigation stack'
        },
        {
            'name': 'RViz Visualization',
            'command': 'ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz',
            'description': 'RViz with Nav2 visualization'
        }
    ]
    
    print_header("Available Commands")
    
    for i, cmd in enumerate(commands, 1):
        print(f"{Colors.BLUE}{i}. {cmd['name']}{Colors.END}")
        print(f"   Description: {cmd['description']}")
        print(f"   Command: {cmd['command']}\n")
    
    print(f"{Colors.YELLOW}To run these in separate terminals, execute each command:${Colors.END}")
    print(f"{Colors.YELLOW}source ~/ros2_ws/install/setup.bash && <command>${Colors.END}\n")

if __name__ == '__main__':
    try:
        run_commands()
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Launcher interrupted.{Colors.END}")
        sys.exit(0)
    except Exception as e:
        print_error(f"An error occurred: {e}")
        sys.exit(1)

