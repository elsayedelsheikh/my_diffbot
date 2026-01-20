my_diffbot_slam_toolbox_online_async.launch.py
=========================================

Launches SLAM Toolbox in online asynchronous mode for simultaneous localization and mapping.

**Use Case**: Build a map of the environment while localizing the robot, useful for
navigation and exploration tasks.

**Launch Command**:

.. code-block:: bash

   ros2 launch my_diffbot_slam my_diffbot_slam_toolbox_online_async.launch.py

**Configuration**:

SLAM configuration is loaded from ``config/slam_toolbox_online_async.yaml``. This file
defines:

* Scan topic
* Map frame and resolution
* SLAM algorithm parameters
* Optimization settings

**Typical Usage**:

Launch the robot, then start SLAM:

.. code-block:: bash

   # Terminal 1: Bring up robot
   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

   # Terminal 2: Start SLAM
   ros2 launch my_diffbot_slam my_diffbot_slam_toolbox_online_async.launch.py

**What it launches**:

* ``slam_toolbox::async_slam_toolbox_node`` - SLAM Toolbox node in online async mode

**Output**:

The SLAM node publishes:

* ``/map`` - Occupancy grid map
* ``/map_metadata`` - Map metadata
* ``/slam_toolbox/scan_visualization`` - Visualization of scan matching
