from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_cfg = PathJoinSubstitution(
        [
            FindPackageShare('merlin_localization'),
            'config',
            'localization_ekf.yaml',
        ]
    )

    localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[params_cfg, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            localization_node,
        ]
    )
