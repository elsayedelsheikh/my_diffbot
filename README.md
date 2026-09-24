# DiffBot

A personal ROS 2 differential drive robot platform used to develop and test Nav2 (Navigation 2) features on ROS2 Rolling. This is a hobby/research repo, not a product.

## Repo Structure

```
my_diffbot/
├── my_diffbot_bringup/            # Launch files, controller configs, top-level bring-up
├── my_diffbot_description/        # URDF/Xacro robot model, meshes, RViz configs
├── my_diffbot_hardware_interface/ # ros2_control hardware interface (RoboAuto ESP32-S3 base)
├── my_diffbot_localization/       # EKF localization config (robot_localization)
├── docker/                        # Dockerfile (base + overlay stages)
├── docker-compose.yaml            # Development container services
├── dependencies.repos             # External repos (ldlidar_stl_ros2)
└── scripts/                       # udev rules and helper scripts
```

## Key Features

**Hardware**
- Differential drive chassis — wheel radius 31 mm, wheel separation 160 mm
- RoboAuto ESP32-S3 base over native USB (`/dev/roboauto`, binary protocol): L298N motor driver,
  hall encoders (515 counts/rev), on-board PID, and a BNO055 9-DOF IMU fused on the MCU
- LD06 360° LIDAR, 8 m range — Jetson UART (`/dev/ttyTHS1`, 230400 baud)

**Software**
- ROS2 with ros2_control, robot_localization (EKF), Nav2, and SLAM Toolbox
- Docker-based development workflow (base + overlay images)

## Supported Distros

- Jazzy
- Kilted
- Rolling

## Device Permissions

Apply the udev rules on the host so the LIDAR and the RoboAuto base get stable symlinks
(`/dev/lidar`, `/dev/roboauto`):

```bash
sudo cp scripts/97-ldlidar.rules scripts/99-roboauto.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The `dev` container mounts the live `/dev`, so the symlinks survive a replug or an ESP32 reset.

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
| `use_imu` | `false` | Fuse the RoboAuto IMU into odometry (EKF) |
| `mcu_serial_port` | `/dev/roboauto` | RoboAuto serial port |
| `mcu_baud_rate` | `115200` | RoboAuto baud rate (ignored by the USB CDC link) |
| `lidar_serial_port` | `/dev/ttyTHS1` | LIDAR serial port |

The IMU is always published on `/imu/data` (`imu_broadcaster`); `use_imu` only adds the EKF.
IMU calibration, temperature and staleness go to `/diagnostics`.

Example with IMU fusion enabled:

```bash
ros2 launch my_diffbot_bringup bringup_robot.launch.py use_imu:=true
```

## Navigation and SLAM

Navigation uses [nav2_bringup](https://github.com/ros-navigation/navigation2) directly — no custom nav package needed.

```bash
ros2 launch nav2_bringup bringup_launch.py \
  slam:=True \
  use_sim_time:=false \
  use_keepout_zones:=False \
  use_speed_zones:=False
```

Visualize with RViz:

```bash
rviz2 -d /home/sayed/Projects/nav2_ws/src/navigation2/nav2_bringup/rviz/nav2_default_view.rviz
```

## RoboAuto Base

The hardware interface talks to the RoboAuto ESP32-S3 firmware over its binary
serial protocol. PID gains (x1000) and the motor watchdog are URDF params,
pushed on activation. Two gpio controllers are exposed at runtime:

```bash
# Onboard RGB LED: led_mode 0 off, 1 solid, 2 blink, 3 alternate; colours are 0xRRGGBB
# (65280 = 0x00FF00: green, blinking every 500 ms)
ros2 topic pub --once /led_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [led], interface_values: [{interface_names: [led_mode, led_color, led_color_alt, led_period_ms], values: [2, 65280, 0, 500]}]}"

# Live PID re-tune (kp=1.0, ki=3.0 per wheel); sent to the MCU only when a value changes
ros2 topic pub --once /roboauto_tuning_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [roboauto_pid], interface_values: [{interface_names: [kp_l, ki_l, kd_l, kp_r, ki_r, kd_r], values: [1000, 3000, 0, 1000, 3000, 0]}]}"
```

Wheel target/measured/firmware velocity (mrps) and PWM duty (‰) are registered
with ros2_control introspection (`left_wheel.target_velocity`, `left_wheel.pwm`, …)
for tuning plots.

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
