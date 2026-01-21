import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_diffbot_navigation')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Use visualization if true',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_dir, 'launch', 'my_diffbot_slam_launch.py')]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_dir, 'launch', 'my_diffbot_navigation_launch.py')]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_dir, 'launch', 'rviz_launch.py')]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(slam)
    ld.add_action(nav2)
    ld.add_action(rviz2)

    return ld
