# hunter2_gazebo/launch/gz_bridge.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridges = [
        # GZ -> ROS
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

        # ROS -> GZ
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        
                # ===== Sensores =====
        # IMU (GZ -> ROS)
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',

        # LiDAR 3D como PointCloud2 (GZ -> ROS)
        '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        
        #LiDaR 2D como LaserScan (GZ -> ROS)
        '/lidar_2d@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    ]

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            arguments=bridges,
            # Se seus tópicos no Gazebo tiverem prefixo (/model/hunter2/...), remapeie:
            remappings=[
            #     ('/cmd_vel', '/model/hunter2/cmd_vel'),
               ('/odom',    '/odom_abs'),
            ]
        )
    ])
