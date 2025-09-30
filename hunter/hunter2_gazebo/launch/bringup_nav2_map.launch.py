# bringup_nav2_map_with_amcl.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Args
    log_level = LaunchConfiguration('log_level')
    declare_log = DeclareLaunchArgument('log_level', default_value='INFO')

    # Pacotes e caminhos
    gz_pkg_share = get_package_share_directory('hunter2_gazebo')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params = os.path.join(gz_pkg_share, 'config', 'nav2_params.yaml')
    map_yaml    = os.path.join(gz_pkg_share, 'map', 'powerstation_raster.yaml')

    # 1) Display (RViz + RSP/JSP)
    display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'gui': 'true'}.items()
    )

    # 2) ros_gz_bridge (clock, odom, cmd_vel, sensores…)
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg_share, 'launch', 'gz_bridge.launch.py')
        )
    )

    # 3) Map Server (fornece /map do arquivo salvo)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
            'frame_id': 'map',
            'topic_name': 'map'
        }]
    )

    # 4) AMCL (publica map->odom)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[nav2_params, {'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    # 5) Lifecycle manager para map_server + amcl
    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # 6) Nav2 (navigation stack). Atenção: navigation_launch.py NÃO sobe map_server.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': nav2_params,
            'slam': 'false',
            # 'map' é ignorado aqui; quem lê o mapa é o map_server acima
        }.items()
    )

    return LaunchDescription([
        declare_log,
        #display,
        bridge,
        map_server,
        amcl,
        lifecycle,
        nav2,
    ])
