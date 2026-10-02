from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

# Mirrors nav2_bringup's navigation_launch.py + localization_launch.py (composition mode), minus
# route/docking servers. nav2_bringup itself is not a dependency: its apt package pulls in
# navigation2 (rviz plugins) and Gazebo, which the headless Jetson doesn't need.
NAV_NODES = [
    ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', True),
    ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server', False),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', False),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', True),
    ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother', True),
    ('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor', 'collision_monitor', False),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', False),
    ('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower', 'waypoint_follower', False),
]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use simulation time',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Map yaml to localize on with AMCL. Leave empty to navigate on '
            'the live map from slam.launch.py.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution(
                [FindPackageShare('merlin_navigation'), 'config', 'nav2_params.yaml']
            ),
            description='Nav2 parameters file',
        ),
    ]

    map_yaml = LaunchConfiguration('map')
    params = [LaunchConfiguration('params_file'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]

    def lifecycle_manager(name, node_names):
        return ComposableNode(
            package='nav2_lifecycle_manager',
            plugin='nav2_lifecycle_manager::LifecycleManager',
            name=name,
            parameters=[{'autostart': True, 'node_names': node_names}],
        )

    # One process for all servers: saves RAM on the Nano.
    # cmd_vel chain: controller/behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
    # -> collision_monitor -> cmd_vel (merlin_base_controller listens there).
    container = ComposableNodeContainer(
        name='nav2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        # Process-wide params file: reaches the costmap nodes the servers create internally.
        parameters=params,
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg,
                plugin=plugin,
                name=name,
                parameters=params,
                remappings=[('cmd_vel', 'cmd_vel_nav')] if nav_cmd_vel else [],
            )
            for pkg, plugin, name, nav_cmd_vel in NAV_NODES
        ]
        + [lifecycle_manager('lifecycle_manager_navigation', [n[2] for n in NAV_NODES])],
    )

    # Saved map: map_server + AMCL provide map and map -> odom. Without one, slam_toolbox does.
    localization = LoadComposableNodes(
        condition=IfCondition(PythonExpression(["'", map_yaml, "' != ''"])),
        target_container='nav2_container',
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=params + [{'yaml_filename': map_yaml}],
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=params,
            ),
            lifecycle_manager('lifecycle_manager_localization', ['map_server', 'amcl']),
        ],
    )

    return LaunchDescription(declared_arguments + [container, localization])
