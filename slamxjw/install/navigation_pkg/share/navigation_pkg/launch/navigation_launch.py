from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value='/home/booster/Workspace/slamxjw/src/maps/rtabmap.yaml',
            description='Full path to map file to load'
        ),


        # 启动 factor_perception
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join('/opt/ros/humble/share/factor_perception/launch', 'factor_perception_launch.py')
            ),
            launch_arguments={'key': '12D0C1E7D1AB466C09BD9AE6427D5240'}.items(),
        ),

        # 启动 nav2_bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join('/opt/ros/humble/share/nav2_bringup/launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'false'
            }.items(),
        ),

        # 启动 rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])
