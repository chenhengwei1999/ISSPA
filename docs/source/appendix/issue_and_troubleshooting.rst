**Issue and Troubleshooting**
=============================

During ISSPA's Compilation
--------------------------

Issue A
~~~~~~~

When running ``catkin_make``, the local environment conflicts with the conda virtual environment.

**Error Message:**

.. code-block:: bash

    -- This workspace overlays: /opt/ros/noetic
    -- Found PythonInterp: /home/iscas/anaconda3/bin/python3 (found suitable version "3.11.5", minimum required is "3") 
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

**Cause Analysis:**

Since there is a Python environment in the locally installed anaconda, ``catkin_make`` uses python in the conda environment by default, which is incompatible with the python version required in the ros environment.

Solution A
~~~~~~~~~~

There are two ways to solve the above problem:

1. In bashrc or zshrc, comment out the ``anaconda`` part, and then reactivate the conda environment when using conda in the future.

2. When using ``catkin_make``, specify the python interpreter, for example ``catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3``.






