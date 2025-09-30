import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    gazebo_world_path = os.path.join(
        get_package_share_directory('hunter2_gazebo'), 'world', 'electrical_substation_v3.world')

    gazebo_options_dict = {
        'world': gazebo_world_path,
        'verbose': 'true'
    }

    gazebo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={'gz_args': ['-r -v4 ', gazebo_world_path], 'on_exit_shutdown': 'true'}.items()
        # dica: se a GUI não abrir, remova listas e use string única:
        # launch_arguments={'gz_args': f'-r -v4 {gazebo_world_path}', 'on_exit_shutdown': 'true'}.items()
    )

    car_sim_options = {
        'start_x': '11',
        'start_y': '36',
        'start_z': '0.36',
        'start_yaw': '-1.5708',
        'pub_tf': 'true',
        'tf_freq': '100.0',
        'blue': 'false'
    }

    spawn_car = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('hunter2_gazebo'),
                'launch', 'hunter_spawn.launch.py')
        ]),
        launch_arguments=car_sim_options.items()
    )




    return LaunchDescription([
        gazebo_simulator,
        spawn_car,
    ])
