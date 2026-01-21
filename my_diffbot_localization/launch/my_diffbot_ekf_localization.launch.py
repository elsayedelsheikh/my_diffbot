from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_cfg = PathJoinSubstitution(
        [
            FindPackageShare('my_diffbot_localization'),
            'config',
            'localization_ekf.yaml',
        ]
    )

    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_cfg],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([localization_node])
