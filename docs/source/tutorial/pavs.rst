:caption:

**PAVS**
=====================================================

The full name of PAVS is Physical Agents Vehicle Small, which is a small 
vehicle for indoor environments, and its basic use will be described in the 
following document.

- Start the Vehicle Chassis and Sensors

- Start the SLAM Program

- Start the Navigation Program

First, the model diagram of the vehicle is shown below:

.. figure:: ../imgs/pavs_structure.jpg
   :alt: PAVS Structure
   :align: center
   :scale: 20%

   **Model Diagram of PAVS**

Connection method
-----------------

There are many ways to connect to a vehicle's internal port, such as through `VNC viewer`, 
`SSH`, `Jupyter Notebook`, etc., or connecting a monitor plugged into the vehicle will work.

When first familiarizing yourself with the vehicle software framework, we recommend that you use a 
monitor plugged into the on-board computing board (e.g., NVIDIA Computer's HDMI connector), which provides a 
visual interface for familiarizing yourself with the internal operating system.

.. note::
    Reserve at least one USB port for the keyboard and mouse kit, which can help you with wireless 
    network configuration and so on.

Once you have configured your network, you can view your IPv4 address via the ``ifconfig`` command. 
After that, you can connect via VNC viewer or SSH (we will use SSH as an example below).

The command line usage of SSH is as follows:

.. code-block:: bash

    ssh <username>@<ipv4_address>

.. note::
    The default username is **jetson**, and the default password is **iscas**.

`Additionally, we prefer that you use VS code for SSH connections during this process.`

As soon as you have connected to the terminal inside the vehicle, you can start some programs 
to run the vehicle.


Start the Vehicle Chassis and Sensors
~~~~~~~~~~~~~~~~~~~~~~~~~

Firstly, launch chassis and sensors driver of the vehicle. 

.. code-block:: bash

    roslaunch pavs_bringup pavs_chassis_and_sensors.launch

With the chassis booted, you can view the current list of messages 
via the ``rostopic list``, e.g. ``/cmd_vel`` is the topic for which the chassis expects twist subscribers.

At this point, launch another terminal, again using SSH to connect to the vehicle, and enter the following 
commands to test that the motors and servos are working properly.

.. code-block:: bash

    rostopic pub /cmd_vel geometry_msgs/Twist "linear:
      x: 0.1
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0" -r 10

If the chassis was successfully activated, the vehicle should have moved forward by now.


Start the SLAM Program
~~~~~~~~~~~~~~~~~~~~~~

After that, you can test if the SLAM program works properly.

.. code-block:: bash

    roslaunch mapping_baselines pavs_map.launch

When the program is started, you can check for message output by typing ``rostopic echo /map`` in the vehicle's 
terminal, which normally outputs a number of matrices containing values from 0 to 1, which represent the probability 
of an obstacle being present in the grid.

Further, you need to control the vehicle movement via a remote controller or a keyboard control node.

.. note::
    Warm reminder: Try not to let the vehicle hit the obstacles during mapping.


When the map is created, you can execute map.sh under the ``~/pa_ws/src/ISSPA/src/isspa_mapping/mapping_baselines/scripts directory`` 
to save your map.

Eventually, the maps will be saved to the ``~/pa_ws/src/ISSPA/src/isspa_mapping/mapping_baselines/maps/`` folder 
with the name `map`.


Start the Navigation Program
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once you have activated the vehicle's chassis and sensors, and you have been given a grid map, it is then 
possible to realize the task of fixed-point navigation!

.. code-block:: bash

    roslaunch navigation_stack pavs_navigation.launch

By now, you will be able to test the effectiveness of the navigation algorithms on ``RVIZ`` by selecting points on the 
map that are free of obstacles.

