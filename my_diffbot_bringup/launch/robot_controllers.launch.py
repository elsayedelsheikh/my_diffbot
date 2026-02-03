from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_mock_hardware',
            default_value='false',
            choices=['true', 'false'],
            description='Start robot with mock hardware mirroring command to its states.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'baud_rate',
            default_value='57600',
            description='Baud rate for the serial communication with the robot.',
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')

    # Get URDF via xacro
    robot_desc_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('my_diffbot_description'),
                    'urdf',
                    'my_diffbot.urdf.xacro',
                ]
            ),
            ' ',
            'use_mock_hardware:=',
            use_mock_hardware,
            ' ',
            'serial_port:=',
            serial_port,
            ' ',
            'baud_rate:=',
            baud_rate,
        ]
    )
    robot_desc_param = ParameterValue(robot_desc_content, value_type=str)
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            {'robot_description': robot_desc_param, 'use_sim_time': use_sim_time}
        ],
    )

    robot_controllers_config = PathJoinSubstitution(
        [
            FindPackageShare('my_diffbot_bringup'),
            'config',
            'base_controllers.yaml',
        ]
    )
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'use_sim_time': use_sim_time}, robot_controllers_config],
        arguments=['--ros-args', '--log-level', log_level],
        output='both',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    range_sensor_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['my_range_sensor_broadcaster'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    robot_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'my_diffbot_base_controller',
            '--param-file',
            robot_controllers_config,
            '--controller-ros-args',
            '-r /my_diffbot_base_controller/cmd_vel:=/cmd_vel',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[robot_controller_spawner],
            )
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        range_sensor_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
