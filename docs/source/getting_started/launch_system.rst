Launch System Guide
===================

This guide documents all available launch files in the My_diffbot ROS 2 workspace, their parameters,
and use cases.

Overview
********

The My_diffbot workspace consists of several packages, each providing launch files for different
functionality:

* **my_diffbot_description** - Robot description and visualization
* **my_diffbot_bringup** - Main robot bringup with controllers
* **my_diffbot_localization** - EKF-based localization
* **my_diffbot_slam** - SLAM Toolbox integration

Launch Files
************

.. toctree::
   :maxdepth: 1

   view_my_diffbot
   my_diffbot_bringup
   my_diffbot_ekf_localization
   my_diffbot_slam

Common Launch Combinations
**************************

Basic Robot Operation
---------------------

.. code-block:: bash

   # Terminal 1: Robot bringup
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: Keyboard control
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true

Robot with Localization
-----------------------

.. code-block:: bash

   # Terminal 1: Robot bringup
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: EKF localization
   ros2 launch my_diffbot_localization my_diffbot_ekf_localization.launch.py

   # Terminal 3: Keyboard control
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true

Robot with SLAM
---------------

.. code-block:: bash

   # Terminal 1: Robot bringup
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: SLAM
   ros2 launch my_diffbot_slam my_diffbot_slam_toolbox_online_async.launch.py

   # Terminal 3: Keyboard control
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true

Full Stack (Robot + Localization + SLAM)
-----------------------------------------

.. code-block:: bash

   # Terminal 1: Robot bringup
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: EKF localization
   ros2 launch my_diffbot_localization my_diffbot_ekf_localization.launch.py

   # Terminal 3: SLAM
   ros2 launch my_diffbot_slam my_diffbot_slam_toolbox_online_async.launch.py

   # Terminal 4: Keyboard control
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true

Tips and Best Practices
***********************

1. **Always source your workspace** before launching:

   .. code-block:: bash

      source ~/my_diffbot_ws/install/setup.bash

2. **Use mock hardware for testing** when you don't have physical hardware available

3. **Check topic lists** to verify everything is running:

   .. code-block:: bash

      ros2 topic list

4. **Monitor robot state** using:

   .. code-block:: bash

      ros2 topic echo /joint_states
      ros2 topic echo /odom

5. **Visualize transforms** using:

   .. code-block:: bash

      ros2 run tf2_tools view_frames

For more advanced usage and customization, see the :ref:`development guides <development_index>` section.
