#!/bin/bash
colcon build
. install/local_setup.bash
ros2 launch crazyflie_ros2_navigation navigation_simulation_launch.py 
