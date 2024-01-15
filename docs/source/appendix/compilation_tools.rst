Compilation Tools
=================

Understanding ROS Build Systems: A Historical Overview
------------------------------------------------------

A build system in ROS (Robot Operating System) refers to a set of functional programs or scripts designed to automate and simplify the compilation 
and linking of source code. ROS build systems, evolving over the years, are built on top of CMake, a meta-build system generating native makefiles 
and workspaces. This article explores the history of ROS build systems, starting with `rosbuild` and culminating in the latest system, `colcon`.

rosbuild (Legacy)
-----------------

`rosbuild` consists of scripts for managing the CMake-based build system in ROS. The CMake wrapper script, `rosmake`, is crucial for building ROS. Most commonly used ROS packages have transitioned to the recommended catkin build system. A package using `rosbuild` is identified by the presence of a `manifest.xml` file, containing information such as cpp cflags and dependencies.

To build packages with `rosbuild`, the command is::

    rosmake <package1> <package2> <package3>

catkin_make
------------

`catkin_make` is a commonly used command for building ROS workspaces. It is a convenient way to invoke calls to CMake and make in the standard CMake workflow. The entire workspace is treated as a single CMake project, making incremental builds faster.

Limitations of `catkin_make` include its inability to process plain CMake packages and the requirement for all targets in the workspace to be unique.

catkin_make_isolated
---------------------

`catkin_make_isolated` improves on `catkin_make` by building isolated workspaces. Each software package is independently configured, built, and installed into its own directory. This allows for the removal of the install artifacts of a single package by deleting its directory.

Building with `catkin_make_isolated` involves creating an isolated ROS catkin workspace, adding the isolated package, and then compiling using::

    catkin_make_isolated

catkin_tools
------------

`catkin_tools`, similar to `catkin_make_isolated`, builds packages in topological order and in parallel where possible, resulting in improved build times. It treats each package as a separate CMake project and catches common build errors using persistent build metadata.

Supported command line verbs include build, clean, configure workspace settings, create packages, run commands with a modified environment, initialize a workspace, list package information, locate workspace directory paths, manage named configuration profiles, test packages, and more.

colcon
------

`colcon` is a command line tool for building, testing, and using multiple software packages in ROS. It supports building different types of packages on different operating systems and is actively developed for both ROS 1 and ROS 2 workspaces.

Building a workspace with `colcon` creates build, install, and log directories. It represents the latest ROS build tool and can handle various package types such as CMake, Python setuptools, cargo, gradle, etc.

Conclusion
-----------

ROS build systems have undergone continuous evolution, from the traditional `rosbuild` to the current state-of-the-art `colcon`, enhancing efficiency and flexibility in software development for robotics.


References
----------

- `ROS Build Systems <https://www.ros.org/reps/rep-0128.html>`_
