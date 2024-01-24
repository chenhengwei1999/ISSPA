**ISSPA Libraries (Python and C++)**
====================================

Some common tools will be integrated into ISSPA in the form of **Python Packages** or **CPP Libraries**, 
the purpose of which is to interconnect ROS packages and give you a clearer insight into how ISSPA works. 
In the meantime, you will also understand ISSPA's program logic and how it relates to each other. Let's start 
following along with the tutorial.

The tutorial includes the following:

- ISSPA Python Packages

    - Creation Method

    - Usage Examples

- ISSPA C++ Libraries

    - Creation Method

    - Usage Examples


ISSPA Python Packages
---------------------

First, you need to understand how to create a Python Package in ROS and how other ROS packages import it. 
Second, you will be introduced to the basic usage of Python Package with code samples.

Creation Method
~~~~~~~~~~~~~~~

Here's an example of creating a python package named ``isspa``, which also has the package name ``isspa``.
Create a ``setup.py`` file in the root directory of the ROS package, i.e., the directory on a level with ``CMakeLists.txt``:

.. code-block:: bash

    mkdir -p ~/ISSPA/src
    cd ~/ISSPA/src
    catkin_create_pkg isspa rospy roscpp std_msgs
    cd isspa
    touch setup.py
    touch scripts/isspa/__init__.py
    touch scripts/isspa/logging_output.py
    chomd +x scripts/isspa/logging_output.py

The contents of the ``setup.py`` file are as follows:

.. code-block:: python

    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup


    d = generate_distutils_setup(
        packages=['isspa'],
        package_dir={'': 'scripts'},
        version='0.1.0'
    )

    setup(**d)

Only two sentences are important here, the rest are the same every time:

.. code-block:: python

    packages=['isspa'],
    package_dir={'': 'scripts'},

The first sentence indicates the name of the package, and the second sentence indicates the directory where the package is located.

.. note::

    The version should be consistent with the version of the package in the ``package.xml`` file.

The contents of the ``logging_output.py`` file are as follows:

.. code-block:: python

    #!/usr/bin/env python
    import rospy

    def logger(msg):
        rospy.loginfo("Welcome to ISSPA. " + msg)


After that, you also need to change the contents of ``CMakeLists.txt``:

.. code-block:: cmake

    ...

    catkin_python_setup()

    catkin_package()

    ...

Finally, you need to build the package:

.. code-block:: bash

    cd ~/ISSPA
    catkin_make
    source devel/setup.bash


If all goes well, you can try running the program in another package. A simple test is as follows:

.. code-block:: bash

    # Open a new terminal
    roscore

    # Open another terminal, and create a script file
    cd  ~/ISSPA/src/
    catkin_create_pkg test_import_python_package rospy roscpp std_msgs
    cd test_import_python_package/scripts
    touch test_import_python_package.py
    chomd +x test_import_python_package.py


Below we add a simple ``test_import_python_package.py`` script under ``~/ISSPA/src/test_import_python_package/scripts`` as an example.

.. code-block:: python
    

    #!/usr/bin/env python
    import rospy
    from isspa import logging_output

    def test_log():
        logging_output.logger("This is a test message.")
        rospy.loginfo("Welcome to ISSPA! This is a message from test_import_python_package.py")

    if __name__ == '__main__':
        rospy.init_node('test_import_python_package')
        test_log()

And, run the following command to make the script executable:

.. code-block:: bash

    # In another terminal
    rosrun test_import_python_package test_import_python_package.py

The following results will be output:

.. code-block:: bash

    [INFO] [1632938655.753961]: Welcome to ISSPA. This is a test message.
    [INFO] [1632938655.758263]: Welcome to ISSPA! This is a message from test_import_python_package.py


Usage Examples
~~~~~~~~~~~~~~

*Needs to be replenished.*


ISSPA C++ Libraries
-------------------

The ROS library configuration for C++ is more complex than for python, so let's learn how to create it!

Creataion Method
~~~~~~~~~~~~~~~~

Here's an example of creating a C++ library named ``isspa``, which also has the head file named ``util.h``.

.. code-block:: bash

    mkdir -p ~/ISSPA/src
    cd ~/ISSPA/src
    catkin_create_pkg isspa rospy roscpp std_msgs
    cd isspa
    mkdir include
    touch include/isspa/util.h

The contents of the ``util.h`` file are as follows:

.. code-block:: cpp

    #ifndef UTIL_H
    #define UTIL_H

    #include "ros/ros.h"

    namespace isspa
    {
        namespace utils {
            void logger(const char* msg);
        }
    } // namespace isspa

    void sayHello();

    #endif // UTIL_H

.. note::

    Don't misspell ``#ifndef`` as ``#ifdef``. , otherwise the contents of the header file will not be found during compilation.

Then create ``util.cpp`` in the ``~/ISSPA/src/isspa/src`` folder with the following contents:

.. code-block:: cpp

    #include "isspa/util.h"
    #include <iostream>

    using namespace std;

    namespace isspa
    {
        namespace utils {
            void logger(const char* msg)
            {
                cout << "[Welcome to ISSPA util] " << msg << endl;
            }
        }
    } // namespace isspa

    void sayHello()
    {
        ROS_INFO("Hello from util.cpp");
    }

After that, you also need to change the contents of ``CMakeLists.txt`` within ``isspa`` package:

.. code-block:: cmake

    cmake_minimum_required(VERSION 3.0.2)
    project(isspa)

    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    )

    catkin_package(
    INCLUDE_DIRS include
    LIBRARIES isspa
    CATKIN_DEPENDS roscpp rospy std_msgs
    )

    include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    )

    add_library(${PROJECT_NAME}
    src/util.cpp
    )

    target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    )

    install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    )

At this point, the other package requires changes to ``CMakeLists.txt`` and ``package.xml``, 
assuming the name of the other ROS package is ``test_import_roscpp_library``:

We need to create a ``main.cpp`` file first.

.. code-block:: bash

    cd ~/ISSPA/src/
    catkin_create_pkg test_import_roscpp_library rospy roscpp std_msgs
    cd test_import_roscpp_library/src
    touch src/main.cpp

A simple example is as follows:

.. code-block:: cpp
    
    // main.cpp
    #include <ros/ros.h>
    #include <isspa/util.h>

    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "test_import_roscpp_library");
        ros::NodeHandle nh;

        isspa::utils::logger("Hello from main.cpp");
        sayHello();

        return 0;
    }

After that, you also need to change the contents of ``CMakeLists.txt`` within ``test_import_roscpp_library`` package:

.. code-block:: cmake

    ...

    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    isspa
    )

    ...

    add_executable(test_import_roscpp_library src/main.cpp)
    target_link_libraries(test_import_roscpp_library ${catkin_LIBRARIES})

    ...

And add a ``<depend>isspa</depend>`` tag inside the ``package.xml``.

Finally, you need to build the package:

.. code-block:: bash

    cd ~/ISSPA
    catkin_make
    source devel/setup.bash

If all goes well, you can try running the program in another package. A simple test is as follows:

.. code-block:: bash

    # Open a new terminal
    roscore

    # In another terminal
    rosrun test_import_roscpp_library test_import_roscpp_library

The following results will be output:

.. code-block:: bash

    [Welcome to ISSPA util] Hello from util.cpp
    [INFO] [1632938655.757961]: Hello from util.cpp


Usage Examples
~~~~~~~~~~~~~~

*Needs to be replenished.*


Reference
---------

- `[ROS] Include a Cpp header from another package <https://roboticsbackend.com/ros-include-cpp-header-from-another-package/>`_

- `[ROS] How To Import a Python Module From Another Package <https://roboticsbackend.com/ros-import-python-module-from-another-package/>`_





