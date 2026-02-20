from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args_declare = [
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyTHS1',
            description='Specifying usb port to connected lidar',
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='lidar_link',
            description='Specifying frame_id of lidar',
        ),
    ]

    lidar_filters_cfg = PathJoinSubstitution(
        [
            FindPackageShare('my_diffbot_bringup'),
            'config',
            'lidar_filters.yaml',
        ]
    )
    lidar_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        remappings=[('/scan', '/scan_raw'), ('/scan_filtered', '/scan')],
        parameters=[lidar_filters_cfg],
    )

    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')

    ld06_lidar = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='ld06_lidar',
        parameters=[
            {
                'product_name': 'LDLiDAR_LD06',
                'topic_name': 'scan_raw',
                'frame_id': frame_id,
                'port_name': serial_port,
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }
        ],
        output='screen',
    )

    return LaunchDescription(args_declare + [ld06_lidar, lidar_filter_node])
