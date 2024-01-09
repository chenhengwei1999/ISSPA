.. _catkin_make_problems_and_solutions:

==============================================
Problems and Solutions Encountered During Catkin Make Process
==============================================

Issue 1: Python environment conflicts
--------------------------------------

When running `catkin_make`, the host default environment conflicts with the virtual environment.

**Error Message:**

.. code-block:: bash

    -- This workspace overlays: /opt/ros/noetic
    -- Found PythonInterp: /yourhost/anaconda3/bin/python3 (found suitable version "3.11.5", minimum required is "3") 
    -- Using PYTHON_EXECUTABLE: /yourhost/anaconda3/bin/python3
    -- Using Debian Python package layout
    -- Could NOT find PY_em (missing: PY_EM) 
    CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
      Unable to find either executable 'empy' or Python module 'em'...  try
      installing the package 'python3-empy'
    Call Stack (most recent call first):
      /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include)
      /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include)
      CMakeLists.txt:58 (find_package)

    -- Configuring incomplete, errors occurred!
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeOutput.log".
    Invoking "cmake" failed

**Reason:**

> Since there is a Python environment in the locally installed anaconda, `catkin_make` uses python in the conda environment by default, which is incompatible with the python version required in the ros environment.

**Solution:(1 or 2)**

1. In bashrc or zshrc, comment out the `anaconda` part, and then reactivate the conda environment when using conda in the future.

2. When `catkin_make`, specify to use python in /usr/bin/, for example `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`

Issue 2: libuvc is not found
---------------------------

When running `catkin_make`, none of the required 'libuvc' found.

**Error Message:**

.. code-block:: bash

    CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:707 (message):
      None of the required 'libuvc' found
    Call Stack (most recent call first):
      isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:35 (pkg_search_module)


    CMake Error at isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:37 (message):
      libuvc is not found


    -- Configuring incomplete, errors occurred!
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeOutput.log".
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeError.log".
    Invoking "cmake" failed

**Solution:**

```bash
sudo apt-get install libuvc-dev

==============================
Problems and Solutions Encountered During Catkin Make Process
==============================

Issue 1: Python environment conflicts
---------------------------------------

When running `catkin_make`, the host default environment conflicts with the virtual environment

**Error Message:**

.. code-block:: bash

    -- This workspace overlays: /opt/ros/noetic
    -- Found PythonInterp: /yourhost/anaconda3/bin/python3 (found suitable version "3.11.5", minimum required is "3") 
    -- Using PYTHON_EXECUTABLE: /yourhost/anaconda3/bin/python3
    -- Using Debian Python package layout
    -- Could NOT find PY_em (missing: PY_EM) 
    CMake Error at /opt/ros/noetic/share/catkin/cmake/empy.cmake:30 (message):
      Unable to find either executable 'empy' or Python module 'em'...  try
      installing the package 'python3-empy'
    Call Stack (most recent call first):
      /opt/ros/noetic/share/catkin/cmake/all.cmake:164 (include)
      /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:20 (include)
      CMakeLists.txt:58 (find_package)

    -- Configuring incomplete, errors occurred!
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeOutput.log".
    Invoking "cmake" failed

**Reason:**

>  Since there is a Python environment in the locally installed anaconda, `catkin_make` uses python in the conda environment by default, which is incompatible with the python version required in the ros environment.

**Solution:(1 or 2)**

1. In bashrc or zshrc, comment out the `anaconda` part, and then reactivate the conda environment when using conda in the future.

2. When `catkin_make`, specify to use python in /usr/bin/, for example `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`



Issue 2: libuvc is not found
------------------------------

When running `catkin_make`, None of the required 'libuvc' found

**Error Message:**

.. code-block:: bash

    CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:707 (message):
      None of the required 'libuvc' found
    Call Stack (most recent call first):
      isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:35 (pkg_search_module)


    CMake Error at isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:37 (message):
      libuvc is not found

    -- Configuring incomplete, errors occurred!
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeOutput.log".
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeError.log".
    Invoking "cmake" failed

**Solution:**

.. code-block:: bash

    sudo apt-get install libuvc-dev



Issue 3: None of the required 'libglog' found
----------------------------------------------

**Error Message:**

.. code-block:: bash

    -- ==> add_subdirectory(isspa_plugin_devices/ros_astra_camera)
    -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
    -- Checking for one of the modules 'libuvc'
    -- Checking for one of the modules 'libglog'
    CMake Error at /usr/share/cmake-3.16/Modules/FindPkgConfig.cmake:707 (message):
      None of the required 'libglog' found
    Call Stack (most recent call first):
      isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:39 (pkg_search_module)


    CMake Error at isspa_plugin_devices/ros_astra_camera/CMakeLists.txt:42 (message):
      glog is not found


    -- Configuring incomplete, errors occurred!
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeOutput.log".
    See also "/yourhost/ISSPA/build/CMakeFiles/CMakeError.log".
    Invoking "cmake" failed

**Solution:**

.. code-block:: bash

    sudo apt-get install libgoogle-glog-dev



Issue 4: Missing package "costmap_2d" 
--------------------------------------

**Error Message:**

.. code-block:: bash

    CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
    Could not find a package configuration file provided by "costmap_2d" with
    any of the following names:

    costmap_2dConfig.cmake
    costmap_2d-config.cmake

**Solution:**

.. code-block:: bash

    sudo apt-get install ros-noetic-costmap-2d



Issue 5: Missing package "nav-core" 
------------------------------------

**Error Message:**

.. code-block:: bash

    -- Using these message generators: gencpp;geneus;genlisp;gennodejs;genpy
    -- Could NOT find nav_core (missing: nav_core_DIR)
    -- Could not find the required component 'nav_core'. The following CMake error indicates that you either need to install the package with the same name or change your environment so that it can be found.
    CMake Error at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
      Could not find a package configuration file provided by "nav_core" with any
      of the following names:

        nav_coreConfig.cmake
        nav_core-config.cmake

**Solution:**

.. code-block:: bash

    sudo apt-get install ros-noetic-nav-core





