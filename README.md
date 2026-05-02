# DiffBot

A personal ROS 2 differential drive robot platform used to develop and test Nav2 (Navigation 2) features on ROS2 Rolling. This is a hobby/research repo, not a product.

## Repo Structure

```
my_diffbot/
├── my_diffbot_bringup/            # Launch files, controller configs, top-level bring-up
├── my_diffbot_description/        # URDF/Xacro robot model, meshes, RViz configs
├── my_diffbot_hardware_interface/ # ros2_control hardware interface (serial MCU)
├── my_diffbot_localization/       # EKF localization config (robot_localization)
├── my_diffbot_navigation/         # Nav2 and SLAM launch files, parameter tuning
├── docker/                        # Dockerfile (base + overlay stages)
├── docker-compose.yaml            # Development container services
├── dependencies.repos             # External repos (ldlidar_stl_ros2, bno055)
└── scripts/                       # udev rules and helper scripts
```

## Key Features

**Hardware**
- Differential drive chassis — wheel radius 31 mm, wheel separation 160 mm
- MCU over serial (`/dev/ttyUSB0`, 57600 baud) with quadrature encoders (500 counts/rev)
- LD06 360° LIDAR, 8 m range — Jetson UART (`/dev/ttyTHS1`, 230400 baud)
- BNO055 9-DOF IMU

**Software**
- ROS2 with ros2_control, robot_localization (EKF), Nav2, and SLAM Toolbox
- Docker-based development workflow (base + overlay images)

## Supported Distros

- Jazzy
- Kilted
- Rolling

## Device Permissions

Apply the udev rule for the LIDAR so its serial port gets a stable symlink:

```bash
sudo cp scripts/97-ldlidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Or grant temporary access directly:

```bash
sudo chmod 666 /dev/ttyUSB0 /dev/ttyTHS1 /dev/i2c-1
```

## Build

```bash
# Import external dependencies
vcs import src < dependencies.repos

# Install ROS deps
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=On
source install/setup.bash
```

## Hardware Bringup

```bash
ros2 launch my_diffbot_bringup bringup_robot.launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `use_imu` | `false` | Enable BNO055 IMU |
| `mcu_serial_port` | `/dev/ttyUSB0` | MCU serial port |
| `mcu_baud_rate` | `57600` | MCU baud rate |
| `lidar_serial_port` | `/dev/ttyTHS1` | LIDAR serial port |

Example with IMU enabled:

```bash
ros2 launch my_diffbot_bringup bringup_robot.launch.py use_imu:=true
```

## Navigation and SLAM

```bash
ros2 launch my_diffbot_navigation nav_bringup_launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `use_rviz` | `false` | Launch RViz |

The nav bringup includes SLAM Toolbox for simultaneous mapping and navigation.

## Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Docker Usage

Two services are defined in `docker-compose.yaml`:

| Service | Image | Purpose |
|---|---|---|
| `base` | `my_diffbot:base` | ROS 2 Kilted base layer |
| `overlay` | `my_diffbot:overlay` | Workspace build on top of base |

Build images:

```bash
docker compose build base
docker compose build overlay
```

Run an interactive shell:

```bash
docker compose run --rm overlay bash
```

## External Dependencies

Managed via `dependencies.repos` and imported with `vcs`:

- **ldlidar_stl_ros2** — LD06 driver (forked at `elsayedelsheikh/ldlidar_stl_ros2`)
- **bno055** — BNO055 IMU driver (`flynneva/bno055`)
