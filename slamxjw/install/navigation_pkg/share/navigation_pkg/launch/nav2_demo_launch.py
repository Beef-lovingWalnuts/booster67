# Requirements:
#   Install Turtlebot3 packages
#   Modify turtlebot3_waffle SDF:
#     1) Edit /opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf
#     2) Add
#          <joint name="camera_rgb_optical_joint" type="fixed">
#            <parent>camera_rgb_frame</parent>
#            <child>camera_rgb_optical_frame</child>
#            <pose>0 0 0 -1.57079632679 0 -1.57079632679</pose>
#            <axis>
#              <xyz>0 0 1</xyz>
#            </axis>
#          </joint> 
#     3) Rename <link name="camera_rgb_frame"> to <link name="camera_rgb_optical_frame">
#     4) Add <link name="camera_rgb_frame"/>
#     5) Change <sensor name="camera" type="camera"> to <sensor name="camera" type="depth">
#     6) Change image width/height from 1920x1080 to 640x480
# Example:
#   $ ros2 launch rtabmap_demos turtlebot3_sim_rgbd_demo.launch.py
#
#   Teleop:
#     $ ros2 run turtlebot3_teleop teleop_keyboard

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import os

def launch_setup(context, *args, **kwargs):

    # Directories
    pkg_nav2_bringup = get_package_share_directory(
        'nav2_bringup')
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare('navigation_pkg'), 'config', 'factor_perception_nav2_params.yaml']
    )
    pkg_factor_perception = get_package_share_directory(
        'factor_perception'
    )

    
    # Paths
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    rviz_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'rviz_launch.py'])
    factor_perception_launch = PathJoinSubstitution(
        [pkg_factor_perception, 'launch', 'factor_perception_launch.py'])

    # Includes
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file),
        ]
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([rviz_launch])
    )

    # 启动 rtabmap
    # rtabmap = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([rtabmap_launch]),
    #     launch_arguments=[
    #         ('localization', LaunchConfiguration('localization')),
    #         ('use_sim_time', 'true')
    #     ]
    # )

    # 启动 factor_perception
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([factor_perception_launch]),
        launch_arguments={'key': '12D0C1E7D1AB466C09BD9AE6427D5240'}.items(),
    )

    return [
        # Nodes to launch
        nav2,
        rviz,
        rtabmap
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])