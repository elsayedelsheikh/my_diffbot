view_my_diffbot.launch.py
===================

Visualizes the robot's URDF model in RViz2 without running controllers or hardware interfaces.

**Use Case**: Quick visualization of the robot model, testing URDF changes, or understanding
the robot's structure.

**Launch Command**:

.. code-block:: bash

   ros2 launch my_diffbot_description view_my_diffbot.launch.py

**Parameters**:

* ``use_sim_time`` (default: ``false``)
  - Whether to use simulation time instead of wall clock time
  - Options: ``true``, ``false``

**Example with simulation time**:

.. code-block:: bash

   ros2 launch my_diffbot_description view_my_diffbot.launch.py use_sim_time:=true

**What it launches**:

* ``robot_state_publisher`` - Publishes robot transforms
* ``joint_state_publisher`` - Publishes joint states (with GUI for manual control)
* ``rviz2`` - Visualization tool with pre-configured view
