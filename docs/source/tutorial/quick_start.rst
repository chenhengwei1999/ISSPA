**Quick Start**
===============

In this tutorial, you will learn how to use the `ISSPA source code <https://github.com/iscas-tis/ISS-PA/>`_
and how PA (Pyhsical Agents) are launched! Specifically, we will guide you on how to setup your system with 
respect to the following topics:

- `Developing Environment`_

- `WorkSpace Setup`_


.. _`Developing Environment`:

Developing Environment
----------------------

We recommend using **Ubuntu 20.04** as your operating system for development, since it is the last version 
that support ROS1 noetic, the version of the Robot Operating System we use at the moment; 
we will upgrade to ROS2 in future.

From now on, we assume you have already installed these components in your machine:

- `Ubuntu 20.04 <https://releases.ubuntu.com/20.04/>`_

- `ROS1 noetic <http://wiki.ros.org/noetic>`_

The following are some of the dependency libraries that you need to install before ISSPA can be compiled and run:

.. code-block:: bash

    sudo apt update
    sudo apt install libuvc-dev libgoogle-glog-dev ros-noetic-costmap-2d ros-noetic-nav-core libceres-dev

You might also need to install ``git`` to clone the `ISSPA <https://github.com/iscas-tis/ISS-PA/>`_ repository:

.. code-block:: bash

    sudo apt install git


.. _`WorkSpace Setup`:

WorkSpace Setup
---------------

`ISSPA <https://github.com/iscas-tis/ISS-PA/>`_ is our main repository; it is recommended that you first understand its directory structure, 
which is necessary for later use and development.

.. image:: ../imgs/github_mark.png
   :target: https://github.com/iscas-tis/ISS-PA/
   :alt: GitHub Repository
   :align: center
   :width: 20%

First, open a terminal and clone the source code inside your preferred directory; we just use the ``/home/$USER/`` directory as an example:

.. code-block:: bash

  cd /home/$USER
  git clone https://github.com/iscas-tis/ISS-PA.git

.. note::

    Make sure that ROS noetic environment is correctly loaded before proceeding with the following steps.
    This can be done just for the current terminal session by means of the command

    .. code-block:: bash

        source /opt/ros/noetic/setup.bash

    Add this command to your ``~/.bashrc`` file to automatically load ROS noetic in each terminal session.


Next, use ``catkin_make`` to compile the workspace.

.. code-block:: bash
  
  cd /home/$USER/ISSPA
  catkin_make

.. note::

    **Hints:** You can also use ``catkin_make_isolated`` or ``catkin`` for compilation. ROS provides a number of compilation tools, the differences and advantages 
    of which are described at :doc:`here </appendix/compilation_tools>`.

When executing ``catkin_make``, you may encounter some problems. 
Instructions on how to manage common **Issues & Troubleshooting** can be found :doc:`here </appendix/issue_and_troubleshooting>`.

.. note::

   **Warm reminder:** Please remember to refresh the environment variables before using the program.
   Alternatively, storing them in ``~/.bashrc`` is fine.

   .. code-block:: bash

      cd /home/$USER/ISSPA
      source devel/setup.bash

      # echo "source ~/ISSPA/devel/setup.bash" >> ~/.bashrc
    
