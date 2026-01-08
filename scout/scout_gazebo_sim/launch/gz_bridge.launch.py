from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridges = [
    # ===== Tempo / Estado =====
    # GZ -> ROS
    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',

    # ===== Comando =====
    # ROS -> GZ
    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',

    # ===== Sensores =====
    # IMU (GZ -> ROS)
    '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',

    # LiDAR 3D -> PointCloud2
    '/lidar_3d/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',

    # LiDAR 2D -> LaserScan
    '/lidar_2d@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',

    # Altímetro
    '/altimeter@ros_gz_interfaces/msg/Altimeter[ignition.msgs.Altimeter',

    # ===== CÂMERA RGB-D =====
    # Imagem RGB
    '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',

    # CameraInfo RGB
    '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',

    # Imagem de profundidade
    '/camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',

    # PointCloud gerada pela RGB-D
    '/camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
    ]


    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            arguments=bridges,
            remappings=[
               ('/odom',    '/odom_abs'),
            ]
        )
    ])
