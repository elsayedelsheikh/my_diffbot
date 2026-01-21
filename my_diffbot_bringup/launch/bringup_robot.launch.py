from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            'use_sensor_fusion',
            default_value='true',
            choices=['true', 'false'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mcu_serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for RoboAuto communication.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'mcu_baud_rate',
            default_value='57600',
            description='Baud rate for RoboAuto communication.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'lidar_serial_port',
            default_value='/dev/ttyTHS1',
            description='Serial port for LIDAR communication.',
        )
    )

    # Launch configuration variables
    use_sensor_fusion = LaunchConfiguration('use_sensor_fusion')
    use_sim_time = LaunchConfiguration('use_sim_time')
    mcu_serial_port = LaunchConfiguration('mcu_serial_port')
    mcu_baud_rate = LaunchConfiguration('mcu_baud_rate')
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')

    # Include launch files
    robot_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('my_diffbot_bringup'),
                        'launch',
                        'robot_controllers.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'log_level': 'info',
            'serial_port': mcu_serial_port,
            'baud_rate': mcu_baud_rate,
        }.items(),
    )

    # LIDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('my_diffbot_bringup'),
                        'launch',
                        'lidar_bringup.launch.py',
                    ]
                )
            ]
        ),
        launch_arguments={'serial_port': lidar_serial_port}.items(),
    )

    # Sensor fusion with robot_localization node
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('my_diffbot_localization'),
                        'launch',
                        'my_diffbot_ekf_localization.launch.py',
                    ]
                )
            ]
        ),
        condition=IfCondition(use_sensor_fusion),
    )

    # IMU BNO055
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare('my_diffbot_bringup'),
                        'launch',
                        'imu_bno055.launch.py',
                    ]
                )
            ]
        )
    )

    launch_files = [
        robot_controllers_launch,
        lidar_launch,
        imu_launch,
        sensor_fusion_launch,
    ]

    return LaunchDescription(declared_arguments + launch_files)
