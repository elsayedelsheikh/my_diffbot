.. _control:

Hardware Interface Development
##############################

Overview
********

My_diffbot implements a custom `ros2_control` hardware interface for its differential-drive system.
The ``My_diffbotSystemHardware`` class provides a ``SystemInterface`` that manages two wheel joints
with velocity commands and position/velocity state feedback.

Architecture
************

The hardware interface is implemented in the ``my_diffbot_ros2_control`` package:

- **Class**: ``My_diffbotSystemHardware`` (inherits from ``hardware_interface::SystemInterface``)
- **Joints**: ``left_wheel_joint`` and ``right_wheel_joint``
- **Command Interface**: Velocity (rad/s)
- **State Interfaces**: Position (rad) and Velocity (rad/s)

Key Methods
***********

Lifecycle Management
  - ``on_init()``: Validates joint configuration and interface requirements
  - ``on_configure()``: Resets joint states and commands
  - ``on_activate()``: Synchronizes commands with current states
  - ``on_deactivate()``: Handles hardware deactivation

Control Loop
  - ``read()``: Reads current joint positions and velocities from hardware
  - ``write()``: Writes velocity commands to hardware

Implementation Notes
*********************

The current implementation includes simulation logic for testing purposes.
In production, replace the mock code in ``read()`` and ``write()`` methods with actual
hardware communication (e.g., serial, CAN, or other protocols).

The interface validates that each joint has:
- Exactly 1 command interface (velocity)
- Exactly 2 state interfaces (position, velocity)

References
**********

Official Documentation
  - `Writing a Hardware Component <https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html>`_
  - `Differential Drive Controller <https://control.ros.org/rolling/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html>`_
  - `Mobile Robot Kinematics <https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html>`_

Workshops & Examples
  - `ROSCon 2022 Workshop <https://github.com/ros-controls/roscon2022_workshop>`_
  - `ROSCon 2023 Workshop <https://github.com/ros-controls/roscon2023_control_workshop>`_
  - `ROSCon 2024 Workshop <https://github.com/ros-controls/roscon2024_control_workshop>`_
  - `ROSCon 2025 Workshop <https://github.com/ros-controls/roscon2025_control_workshop>`_
