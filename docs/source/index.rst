.. My_diffbot documentation master file, created by
   sphinx-quickstart on Wed Dec  3 15:27:37 2025.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

My_diffbot Documentation
==================

Welcome to the My_diffbot documentation! My_diffbot is a ROS 2-based differential-drive robot platform
designed for research, development, and educational purposes.

This documentation provides comprehensive guides for users and developers working with the My_diffbot
robot system, including setup instructions, launch system documentation, and development guides.

What is My_diffbot?
*************

My_diffbot is a modular ROS 2 robot platform featuring:

* **Differential-drive base** with ``ros2_control`` hardware interface
* **Modular architecture** with separate packages for description, bringup, localization, and SLAM
* **Simulation support** with Gazebo integration
* **Localization** using Extended Kalman Filter (EKF)
* **SLAM capabilities** with SLAM Toolbox integration

Quick Links
***********

* :doc:`getting_started/index` - Get up and running with My_diffbot in minutes
* :doc:`getting_started/launch_system` - Comprehensive guide to all launch files and their parameters
* :doc:`development/index` - Development guides for extending and customizing My_diffbot

Documentation Structure
***********************

.. toctree::
   :maxdepth: 2
   :caption: User Guides

   getting_started/index
   getting_started/launch_system

.. toctree::
   :maxdepth: 2
   :caption: Development Guides

   development/index

Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`
