from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    use_sim_time_declare = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use simulation time",
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("my_diffbot_bringup"), "rviz", "config.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            use_sim_time_declare,
            rviz_node,
        ]
    )
