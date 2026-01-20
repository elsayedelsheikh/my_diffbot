my_diffbot.launch.py
==============

Main launch file for bringing up the complete robot system with controllers and hardware interface.

**Use Case**: Primary launch file for running the robot with real or mock hardware, testing
controllers, and visualizing robot state.

**Launch Command**:

.. code-block:: bash

   ros2 launch my_diffbot_bringup my_diffbot.launch.py

**Parameters**:

* ``use_rviz`` (default: ``true``)
  - Automatically start RViz2 with the launch file
  - Options: ``true``, ``false``

* ``use_mock_hardware`` (default: ``false``)
  - Use mock hardware interface instead of real hardware
  - Mock hardware mirrors command velocities to state velocities
  - Options: ``true``, ``false``

* ``use_sim_time`` (default: ``false``)
  - Use simulation time instead of wall clock time
  - Options: ``true``, ``false``

**Examples**:

Launch with mock hardware (for testing):

.. code-block:: bash

   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

Launch without RViz2 (headless operation):

.. code-block:: bash

   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_rviz:=false

Launch for simulation:

.. code-block:: bash

   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true use_sim_time:=true

**What it launches**:

* ``ros2_control_node`` - Controller manager node
* ``robot_state_publisher`` - Publishes robot transforms
* ``joint_state_broadcaster`` - Controller for publishing joint states
* ``my_diffbot_base_controller`` - Differential drive base controller
* ``rviz2`` - Visualization (if ``use_rviz:=true``)

**Topic Remapping**:

The base controller's ``cmd_vel`` topic is remapped to ``/cmd_vel`` for compatibility with
standard ROS 2 navigation and teleop tools.
