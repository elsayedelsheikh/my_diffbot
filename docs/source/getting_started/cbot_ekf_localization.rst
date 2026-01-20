my_diffbot_ekf_localization.launch.py
================================

Launches the Extended Kalman Filter (EKF) localization node for robot pose estimation.

**Use Case**: Fuse odometry and sensor data (IMU, wheel encoders) to provide accurate
robot pose estimation.

**Launch Command**:

.. code-block:: bash

   ros2 launch my_diffbot_localization my_diffbot_ekf_localization.launch.py

**Configuration**:

The EKF configuration is loaded from ``config/localization_ekf.yaml``. This file defines:

* Input topics (odometry, IMU, etc.)
* Output topics (filtered odometry, pose)
* EKF filter parameters (process noise, measurement noise, etc.)

**Typical Usage**:

Launch the robot first, then launch localization:

.. code-block:: bash

   # Terminal 1: Bring up robot
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: Start localization
   ros2 launch my_diffbot_localization my_diffbot_ekf_localization.launch.py

**What it launches**:

* ``robot_localization::ekf_node`` - EKF filter node for pose estimation
