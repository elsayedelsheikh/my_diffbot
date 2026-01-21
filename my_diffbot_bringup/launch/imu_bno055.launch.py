import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('my_diffbot_bringup'), 'config', 'imu_bno055.yaml'
    )

    node = Node(
        package='bno055',
        executable='bno055',
        parameters=[config],
        remappings=[
            ('/imu/imu', '/imu/data'),
            ('/imu/imu_raw', '/imu/data_raw'),
        ],
    )
    ld.add_action(node)
    return ld
