*********
files: boosterxjw  slamxjw
description： descripe how to use ros2 and Nav2 to control booster robot
date：     2025.5.30  
author： 丁胜利 

1. run rpc_client_node to control robot by sending command "move(Vx, Vy, Vr)"
source /home/booster/Workspace/boosterxjw/booster_ros2_interface//install/setup.bash
ros2 run booster_rpc_client rpc_client_node

2.launch nav2 to tell robot where to go
cd Workspace/slamxjw/
source install/setup.bash
ros2 launch navigation_pkg nav2_demo_launch.py
*********
