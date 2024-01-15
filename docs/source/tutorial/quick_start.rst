**Quick Start**
======================

In this tutorial, you will learn how to use the `ISSPA source code <https://github.com/chenhengwei1999/ISSPA>`_
and how PA (Pyhsical Agents) are launched! Specifically, the following sections are included:

- Developing Environment

- WorkSpace Setup

- Usage Guide


Developing Environment
----------------------

We recommend using **Ubuntu 20.04** as your development system, ROS1 noetic is the version we use at the moment, 
and we will upgrade to ROS2 later in our work.

- Ubuntu 20.04

- ROS1 noetic

The following are some of the dependent libraries that need to be installed before the program can be compiled and run:

.. code-block:: bash

    sudo apt update
    sudo apt install libuvc-dev libgoogle-glog-dev ros-noetic-costmap-2d ros-noetic-nav-core libceres-dev

WorkSpace Setup
----------------

`ISSPA <https://github.com/chenhengwei1999/ISSPA/>`_ as our main repository, it is recommended that you first understand its directory structure, 
which is necessary for later use and development.

First, clone the source code to the ``/home/$USER/`` directory:

.. code-block:: bash

  cd /home/$USER
  git clone https://github.com/chenhengwei1999/ISSPA.git

Next, use ``catkin_make`` to compile the workspace.

.. code-block:: bash
  
  cd /home/$USER/ISSPA
  catkin_make

.. note::

    **Hints:** You can also select ``catkin_make_isolated`` for compilation. ROS provides a number of compilation tools, the differences and advantages 
    of which are described :doc:`here </appendix/compilation_tools>`.

When executing ``catkin_make``, you may encounter the following problems. Related **Issues & Troubleshooting** can be found :doc:`here </appendix/issue_and_troubleshooting>`.

.. note::

   Warm reminder: Before using the program, remember to refresh the environment variables.
   Or, storing them in ``.bashrc`` is okay.

   .. code-block:: bash

      cd /home/$USER/ISSPA
      source devel/setup.bash

      # echo "source ~/ISSPA/devel/setup.bash" >> ~/.bashrc
    

Usage Guide for PA
------------------------------------

Once the workspace setup is complete, you can attempt to perform the following tasks, 
including vehicle chassis start, sensor start, remote control, SLAM, and navigation. 

When the WorkSpace Setup is completed, you can try the following tasks, 
which contain vehicle chassis startup, sensor startup, remote control, SLAM, 
and navigation.

The following experimental vehicles are currently supported:

PAVS User Manual
~~~~~~~~~~~~~~~~

Refer :doc:`/appendix/pavs_user_manual` for documentation.

Issue & Troubleshooting
-----------------------

In the process of compiling the program and using the hardware there may be many problems, some common problems can be referred to this document 
to try to solve, if you do not find a solution at :doc:`here </appendix/issue_and_troubleshooting>`, you can leave a issue in the ISSPA repository.



