#!/bin/bash

colcon build --symlink-install 
source install/setup.bash
ros2 launch control_1 rsp.launch.py