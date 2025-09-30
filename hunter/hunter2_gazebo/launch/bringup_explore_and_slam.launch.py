# bringup_explore_and_slam.launch.py (trecho)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    gz_pkg_share   = get_package_share_directory('hunter2_gazebo')
    nav2_params    = os.path.join(gz_pkg_share, 'config', 'nav2_params.yaml')
    slam_params    = os.path.join(gz_pkg_share, 'config', 'slam_toolbox_params.yaml')

    # SLAM Toolbox (online async)
    slam_async = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params,   # seu YAML abaixo
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'slam': 'true',
        }.items()
    )
    
    return LaunchDescription([slam_async, 
                              nav2
                              ])
