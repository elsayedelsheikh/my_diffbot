Docker Guide
#################

.. code-block:: bash
   docker compose up bringup-robot-base-controllers


To rebuild the workspace
.. code-block:: bash
   docker compose build dev


For a shell into the container
.. code-block:: bash
   docker compose up -d dev
   docker compose down

In another terminal
.. code-block:: bash
   docker compose exec dev bash


teleop
.. code-block:: bash
   ros2 launch teleop_twist_joy teleop-launch.py publish_stamped_twist:=true joy_config:=xbox
   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.2 -p stamped:=true
