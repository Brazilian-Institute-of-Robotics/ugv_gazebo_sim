import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    gazebo_world_path = os.path.join(
        get_package_share_directory('scout_gazebo_sim'), 'worlds', 'electrical_substation_v3.world')
    
    gz_pkg_share = get_package_share_directory('scout_gazebo_sim')

    gazebo_options_dict = {
        'world': gazebo_world_path,
        'verbose': 'true'
    }

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', gazebo_world_path], 'on_exit_shutdown': 'true'}.items()
    )
    
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items()
    )
    
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, 'launch', 'gz_bridge.launch.py')
        )
    )

    car_sim_options = {
        'start_x': '11',
        'start_y': '36',
        'start_z': '0.0',
        'start_yaw': '-1.57',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                         get_package_share_directory('scout_gazebo_sim'),
                         'launch', 'scout_v2_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )

    return LaunchDescription([
        gazebo_simulator,
        spawn_car,
        #bridge,
        #display,
    ])
