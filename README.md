# Merlin

A personal ROS 2 differential drive robot platform used to develop and test Nav2 (Navigation 2) features on ROS2 Rolling. This is a hobby/research repo, not a product.

## Repo Structure

```
my_diffbot/
├── merlin_bringup/            # Launch files, controller configs, top-level bring-up
├── merlin_description/        # URDF/Xacro robot model, meshes, RViz configs
├── merlin_hardware_interface/ # ros2_control hardware interface (Kestrel ESP32-S3 base)
├── merlin_localization/       # EKF localization config (robot_localization)
├── merlin_navigation/         # Nav2 + slam_toolbox params and thin launch wrappers
├── docker/                    # Dockerfile (base + overlay stages)
├── docker-compose.yaml        # Development container services
├── dependencies.repos         # External repos (ldlidar_stl_ros2)
└── scripts/                   # udev rules and helper scripts
```

## Key Features

**Hardware**
- Differential drive chassis — wheel radius 30 mm, wheel separation 255 mm (centre to centre)
- Kestrel ESP32-S3 base over native USB (`/dev/ttyACM0`, binary protocol): L298N motor driver,
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

Apply the udev rules on the host so the LIDAR gets a stable symlink (`/dev/lidar`) and the
Kestrel base (`/dev/ttyACM0`) is accessible to the `dialout` group:

```bash
sudo cp scripts/97-ldlidar.rules scripts/99-kestrel.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The `dev` container mounts the live `/dev`, so the devices survive a replug or an ESP32 reset.

## WiFi Watchdog

One failed WPA handshake makes NetworkManager block autoconnect for the WiFi profile (there is no
secret agent on the headless Jetson). A timer re-activates it when `wlan0` sits disconnected with the
network in range (override `WIFI_IFACE` / `WIFI_CON` in the service if needed):

```bash
sudo install -m 755 scripts/wifi-watchdog/wifi-watchdog.sh /usr/local/bin/wifi-watchdog
sudo cp scripts/wifi-watchdog/wifi-watchdog.{service,timer} /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now wifi-watchdog.timer
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
ros2 launch merlin_bringup bringup_robot.launch.py
```

Key arguments:

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `mcu_serial_port` | `/dev/ttyACM0` | Kestrel serial port |
| `mcu_baud_rate` | `115200` | Kestrel baud rate (ignored by the USB CDC link) |
| `lidar_serial_port` | `/dev/ttyTHS1` | LIDAR serial port |

The IMU is published on `/imu/data` (`imu_broadcaster`), with calibration, temperature and
staleness on `/diagnostics`. The EKF (`robot_localization`) always runs: it fuses
`/merlin_base_controller/odom` with `/imu/data`, publishes `/odom`, and owns the
`odom -> base_footprint` TF (diff_drive's own TF is disabled). It is started by
`robot_controllers.launch.py`, so it runs with the controllers on their own too.

## Navigation and SLAM

`merlin_navigation` owns the params (copied from upstream Jazzy, then tuned for Merlin). `slam.launch.py`
includes `slam_toolbox/online_async_launch.py`; `navigation.launch.py` mirrors nav2_bringup's composed bringup
without depending on `nav2_bringup`/`navigation2`, which pull RViz and Gazebo onto the headless Jetson.
Everything runs on the Jetson, next to `robot-bringup`. The dev machine visualizes and sends goals:

```bash
rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

```bash
# Map: drive around with teleop, then save
ros2 launch merlin_navigation slam.launch.py
ros2 run nav2_map_server map_saver_cli -f merlin_navigation/maps/<name>

# Navigate while mapping (alongside slam.launch.py)
ros2 launch merlin_navigation navigation.launch.py

# Navigate on a saved map with AMCL
ros2 launch merlin_navigation navigation.launch.py map:=<path>/maps/<name>.yaml
```

The same through Docker: `docker compose up slam` and
`MAP=/overlay_ws/src/merlin_navigation/maps/<name>.yaml docker compose up navigation`.

## Kestrel Base

The hardware interface talks to the Kestrel ESP32-S3 firmware over its binary
serial protocol. PID gains (x1000) and the motor watchdog are URDF params,
pushed on activation. Two gpio controllers are exposed at runtime:

```bash
# Onboard RGB LED: led_mode 0 off, 1 solid, 2 blink, 3 alternate; colours are 0xRRGGBB
# (65280 = 0x00FF00: green, blinking every 500 ms)
ros2 topic pub --once /led_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [led], interface_values: [{interface_names: [led_mode, led_color, led_color_alt, led_period_ms], values: [2, 65280, 0, 500]}]}"

# Live PID re-tune (kp=1.5, ki=3.0 per wheel, the defaults); sent to the MCU only when a value changes
ros2 topic pub --once /kestrel_tuning_controller/commands control_msgs/msg/DynamicInterfaceGroupValues \
  "{interface_groups: [kestrel_pid], interface_values: [{interface_names: [kp_l, ki_l, kd_l, kp_r, ki_r, kd_r], values: [1500, 3000, 0, 1500, 3000, 0]}]}"
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
| `base` | `merlin:base` | ROS 2 Kilted base layer |
| `overlay` | `merlin:overlay` | Workspace build on top of base |

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
