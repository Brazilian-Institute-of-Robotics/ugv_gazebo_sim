import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros.descriptions as ros_desc
import launch.substitutions as subs


def generate_launch_description():
    # Pacotes/base de caminhos
    urdf_pkg_share = FindPackageShare(package='hunter2_base').find('hunter2_base')
    gz_pkg_share = FindPackageShare(package='hunter2_gazebo').find('hunter2_gazebo')

    # URDF/Xacro do robô
    urdf_file = os.path.join(urdf_pkg_share, 'urdf', 'hunter2_base_gazebo.xacro')

    # (Ignition/GZ) Recursos para worlds/models do seu pacote
    # Incluí o diretório do próprio pacote + subpastas padrão
    ign_res = ':'.join([
        gz_pkg_share,
        os.path.join(gz_pkg_share, 'models'),
        os.path.join(gz_pkg_share, 'world'),
    ])

    # Args
    start_x = DeclareLaunchArgument('start_x', default_value='0.0', description='X start')
    start_y = DeclareLaunchArgument('start_y', default_value='0.0', description='Y start')
    start_z = DeclareLaunchArgument('start_z', default_value='0.0', description='Z start')
    start_yaw = DeclareLaunchArgument('start_yaw', default_value='0.0', description='Yaw start (rad)')
    robot_name = DeclareLaunchArgument('robot_name', default_value='', description='Robot name/prefix')

    # ENV para Ignition/Fortress
    set_ign = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=ign_res)
    set_gz  = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH',  value=ign_res)

    # Publica /robot_description a partir do Xacro
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ros_desc.ParameterValue(
                subs.Command([
                    'xacro ', urdf_file,
                    ' robot_name:=', LaunchConfiguration('robot_name')
                ]),
                value_type=str
            )
        }]
    )

    # Spawn no Gazebo (Ignition) a partir do /robot_description
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', LaunchConfiguration('robot_name'),
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('start_x'),
            '-y', LaunchConfiguration('start_y'),
            '-z', LaunchConfiguration('start_z'),
            '-Y', LaunchConfiguration('start_yaw'),
        ],
    )
    
        # === Spawner do joint_state_broadcaster (ros2_control) ===
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],)
    
        # === robot_localization (EKF) ===
    ekf_yaml = os.path.join(gz_pkg_share, 'config', 'ekf_localization.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_yaml, {'use_sim_time': True}],
        # Se quiser mudar o tópico de saída:
        # remappings=[('odometry/filtered', 'odom/filtered')],
    )

    return LaunchDescription([
        start_x, start_y, start_z, start_yaw, robot_name,
        set_ign, set_gz,
        state_pub,
        spawn_entity,
        jsb_spawner,
        ekf_node,
    ])
