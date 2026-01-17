import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory("my_diffbot_description")

    # Path to xacro file
    xacro_file = os.path.join(pkg_share, "urdf", "my_diffbot.urdf.xacro")

    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_share, "rviz", "config.rviz")

    # Process the xacro file to generate URDF
    robot_desc_content = Command(["xacro ", xacro_file])
    robot_desc = ParameterValue(robot_desc_content, value_type=str)

    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_sim_time}],
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
