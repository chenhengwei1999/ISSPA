**Quick Start**
======================

In this tutorial, you will learn how to use the `ISSPA source code <https://github.com/chenhengwei1999/ISSPA>`_
and how PA (Pyhsical Agent) is launched! Specifically, the following sections are included:

- Developing Environment

- WorkSpace Setup

- Usage Guide


Developing Environment
----------------------

We recommend using **Ubuntu** as your development system, ROS1 noetic is the main version we use at the moment, 
and we will upgrade to ROS2 later in our work.

- Ubuntu 20.04

- ROS1 noetic


WorkSpace Setup
----------------

ISSPA as our main repository, it is recommended that you first understand its directory structure, 
which is necessary for later use and development.

- First, create a workspace in the ``/home/$USER/`` directory:
  
  .. code-block:: bash
    
    mkdir ~/pa_ws/src -p

- Next, put the source program inside ``src``:

  .. code-block:: bash

    cd ~/pa_ws/src
    git clone https://github.com/chenhengwei1999/ISSPA.git

- Finally, use ``catkin_make`` to compile the workspace.
  
  .. code-block:: bash
    
    catkin_make

When executing ``catkin_make``, you may encounter the following environment adaptation problems. 
The detailed solutions you can see :doc:`issue of catkin_make </tutorial/problems_catkin_make>`.

.. note::

   **Warm reminder:** Before using the program, remember to refresh the environment variables.
   Or, storing them in ``.bashrc`` is okay.

   .. code-block:: bash

      source devel/setup.bash

      # echo "source ~/pa_ws/devel/setup.bash" >> ~/.bashrc
    

Usage Guide for PA
------------------------------------

Once the workspace setup is complete, you can attempt to perform the following tasks, 
including vehicle chassis start, sensor start, remote control, SLAM, and navigation. 

When the WorkSpace Setup is completed, you can try the following tasks, 
which contain vehicle chassis startup, sensor startup, remote control, SLAM, 
and navigation.

The following experimental vehicles are currently supported:

PAVS
~~~~

Refer :doc:`/tutorial/pavs` for documentation.

