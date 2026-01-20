Getting Started
================

This guide will help you get started with My_diffbot, from building the workspace to running your
first robot simulation.

Prerequisites
*************

Before you begin, ensure you have the following installed:

* **ROS 2 Jazzy Jalisco** (or compatible version)
* **Python 3** (Python 3.8 or higher)
* **colcon** build tool
* **xacro** (for processing URDF files)

Install ROS 2 dependencies:

.. code-block:: bash

   sudo apt update
   sudo apt install -y \
     ros-jazzy-ros2-control \
     ros-jazzy-ros2-controllers \
     ros-jazzy-joint-state-publisher \
     ros-jazzy-robot-state-publisher \
     ros-jazzy-rviz2 \
     ros-jazzy-teleop-twist-keyboard \
     ros-jazzy-robot-localization \
     ros-jazzy-slam-toolbox \
     python3-xacro

Building the Workspace
**********************

1. **Create a workspace** (if you haven't already):

   .. code-block:: bash

      mkdir -p ~/my_diffbot_ws/src
      cd ~/my_diffbot_ws/src

2. **Clone the repository**:

   .. code-block:: bash

      git clone <your_repository_url> my_diffbot-ros
      cd ~/my_diffbot_ws

3. **Install dependencies**:

   .. code-block:: bash

      rosdep update
      rosdep install --from-paths src --ignore-src -r -y

4. **Build the workspace**:

   .. code-block:: bash

      source /opt/ros/jazzy/setup.bash
      colcon build --symlink-install

5. **Source the workspace**:

   .. code-block:: bash

      source install/setup.bash

   .. tip::
      Add this to your ``~/.bashrc`` to automatically source the workspace:

      .. code-block:: bash

         echo "source ~/my_diffbot_ws/install/setup.bash" >> ~/.bashrc

Quick Start
***********

View Robot Description
----------------------

Start by visualizing the robot's URDF model in RViz2:

.. code-block:: bash

   ros2 launch my_diffbot_description view_my_diffbot.launch.py

This will:

* Load the My_diffbot URDF description
* Start the robot state publisher
* Launch RViz2 with a pre-configured visualization

You can interact with the robot model using the joint state publisher GUI to see how
the joints move.

Bring Up the Robot (Mock Hardware)
-----------------------------------

To bring up the robot with mock hardware (for testing without physical hardware):

.. code-block:: bash

   ros2 launch my_diffbot_bringup my_diffbot.launch.py use_mock_hardware:=true

This launch file will:

* Start the ``ros2_control`` hardware interface
* Spawn the joint state broadcaster
* Spawn the base controller
* Launch RViz2 with robot visualization

.. note::
   The mock hardware mode mirrors command velocities to state velocities, allowing you
   to test the control system without physical hardware.

Control the Robot
-----------------

Once the robot is running, you can control it using keyboard teleop in a new terminal:

.. code-block:: bash

   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true

This will allow you to:

* Move the robot forward/backward with ``i`` and ``,`` keys
* Turn left/right with ``j`` and ``l`` keys
* Adjust speed with ``q``/``z`` and ``w``/``x`` keys

For more information about ``teleop_twist_keyboard``, see the
`official documentation <https://docs.ros.org/en/jazzy/p/teleop_twist_keyboard/>`_.

Next Steps
**********

Now that you have the basic robot running, you can:

* Explore the :doc:`launch_system` guide to learn about all available launch files
* Add localization with EKF (see :doc:`my_diffbot_ekf_localization` for details)
* Enable SLAM capabilities (see :doc:`my_diffbot_slam` for details)
* Check out the :ref:`development guides <development_index>` to customize and extend My_diffbot

Troubleshooting
***************

Common Issues
-------------

**Issue**: ``colcon build`` fails with missing dependencies

**Solution**: Make sure you've run ``rosdep install --from-paths src --ignore-src -r -y``

**Issue**: Launch file not found

**Solution**: Ensure you've sourced the workspace: ``source install/setup.bash``

**Issue**: RViz2 shows no robot model

**Solution**: Check that the robot state publisher is running and publishing to ``/robot_description``

For more help, check the :ref:`development guides <development_index>` section or refer to the ROS 2 documentation.
