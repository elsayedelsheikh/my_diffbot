# my_diffbot_description

## Overview

The `my_diffbot_description` package contains the complete robot description for the My_diffbot mobile robot platform. This package defines the robot's physical structure, visual appearance, sensors, and control interfaces using URDF/Xacro format for use in ROS 2 and Gazebo Fortress simulation.

## Robot Features

The My_diffbot is a differential drive mobile robot equipped with:

- **Differential Drive Base**: Two powered wheels with three caster wheels for stability
- **2D LIDAR Sensor**: 360-degree scanning laser rangefinder (8m range)
- **RealSense D435 Camera**: RGB-D camera for vision and depth sensing
- **9 Ultrasonic Sensors**: Arranged in a circular pattern for proximity detection (4m range)

### Physical Specifications

- **Base Mass**: 49.6 kg
- **Wheel Mass**: 5.17 kg each
- **Wheel Radius**: 82.55 mm
- **Wheel Separation**: 210 mm
- **Caster Mass**: 0.038 kg each
- **Camera Mass**: 0.564 kg

## Package Structure

```
my_diffbot_description/
├── CMakeLists.txt           # Build configuration
├── package.xml              # Package metadata and dependencies
├── config/                  # Configuration files (currently empty)
├── launch/
│   └── display_description.launch.py  # Launch file for visualizing robot in RViz
├── meshes/                  # 3D mesh files (.stl)
│   ├── aluminum_hand_part_1.stl
│   ├── aluminum_middle_part2_1.stl
│   ├── base_link.stl
│   ├── caster1_1.stl, caster2_1.stl, caster3_1.stl
│   ├── left_wheel_1.stl, right_wheel_1.stl
│   ├── plastic_down_part_1.stl, plastic_up_part_1.stl
│   ├── robot_base.stl
│   └── z_camera_1.stl
├── rviz/
│   └── config.rviz         # RViz configuration file
└── urdf/
    ├── my_diffbot.urdf.xacro                # Main robot assembly file
    ├── my_diffbot_description.xacro         # Robot structure (links and joints)
    ├── my_diffbot_gz_sim.xacro              # Gazebo-specific plugins and sensors
    └── my_diffbot_ros2_control.urdf.xacro   # ros2_control hardware interface
```

## URDF/Xacro Files

### my_diffbot.urdf.xacro
Main entry point that includes all other xacro files and instantiates the complete robot model. It conditionally includes either Gazebo simulation plugins or ros2_control based on the `sim_gazebo` argument.

### my_diffbot_description.xacro
Defines the physical robot structure including:
- Base link with multiple visual components (aluminum, plastic parts)
- Left and right wheel links with continuous joints
- Three caster wheels for stability
- LIDAR link
- Camera link
- Nine ultrasonic sensor links (ray_link1-9) positioned around the robot
- All inertial properties and collision geometries
- Material definitions for visual appearance

### my_diffbot_gz_sim.xacro
Contains Gazebo-specific configurations:
- **Differential Drive Plugin**: Controls the two-wheel drive system
  - Wheel separation: 0.210 m
  - Wheel radius: 0.08255m
  - Odometry publishing at 50Hz
- **LIDAR Sensor Plugin**: GPU-accelerated 2D laser scanner
  - 360 samples (1-degree resolution)
  - Range: 0.12m - 8.0m
  - Update rate: 10Hz
- **Ultrasonic Sensor Macro**: Reusable sensor definition
  - Range: 0.02m - 4.0m
  - 30-degree cone (±30° horizontal)
  - Update rate: 5Hz

### my_diffbot_ros2_control.urdf.xacro
Defines the ros2_control hardware interface:
- Mock hardware interface for testing (when `use_mock_hardware:=true`)
- Real hardware interface support (DiffBotSystemHardware)
- Velocity command interfaces for both wheels (-16 to +16 rad/s)
- Position and velocity state interfaces
- Only loaded when `sim_gazebo:=false` (for real robot deployment)

## Dependencies

- `ament_cmake` - Build system
- `robot_state_publisher` - Publishes robot state to TF
- `joint_state_publisher` - Publishes joint states
- `joint_state_publisher_gui` - GUI for manual joint control
- `rviz2` - 3D visualization
- `xacro` - XML macro processing
- `ros_gz_sim` - Gazebo Fortress integration
- `ros_gz_bridge` - ROS-Gazebo communication bridge

## Usage

### Visualize Robot in RViz

To visualize the robot description in RViz without simulation:

```bash
ros2 launch my_diffbot_description display_description.launch.py
```

This will:
- Launch the robot state publisher
- Start the joint state publisher GUI (for manual joint control)
- Open RViz with the robot model visualization

### Using with Simulation

For simulation, use the `my_diffbot_gazebo` package instead:

```bash
ros2 launch my_diffbot_gazebo my_diffbot_gz_sim.launch.py
```

## Launch Arguments

### display_description.launch.py

- `use_sim_time` (default: `false`): Use simulation clock

## Topics and Interfaces

When used with Gazebo simulation, the robot provides the following interfaces:

### Published Topics
- `/odom` (nav_msgs/Odometry): Odometry data from wheel encoders
- `/tf` (tf2_msgs/TFMessage): Transform tree
- `/scan` (sensor_msgs/LaserScan): LIDAR scan data
- `/ultrasonic_sensor_[1-9]` (sensor_msgs/Range): Ultrasonic sensor data

### Subscribed Topics
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands for robot motion

## Configuration Notes

- All mesh files use a scale factor of 0.001 (converting from mm to m)
- The robot uses the differential drive kinematics model
- Casters are modeled with friction to allow smooth rolling
- LIDAR is mounted on the upper part of the robot base
- Ultrasonic sensors are distributed at 40-degree intervals around the robot
- The main xacro file uses conditional includes based on `sim_gazebo` argument to switch between simulation and real hardware configurations

## Development

To modify the robot description:

1. Edit the appropriate xacro file(s)
2. Rebuild the package: `colcon build --packages-select my_diffbot_description`
3. Source the workspace: `source install/setup.bash`
4. Test in RViz or Gazebo

## Version

- Package Version: 0.0.1
- ROS 2 Distribution: Compatible with ROS 2 Humble or newer
- Gazebo Version: Gazebo Fortress (Ignition Gazebo)
