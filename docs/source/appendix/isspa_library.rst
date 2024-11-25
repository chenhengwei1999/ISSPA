**ISSPA Libraries (Python and C++)**
====================================

Some common tools will be integrated into ISSPA in the form of **Python Packages** or **CPP Libraries**, 
the purpose of which is to interconnect ROS packages and give you a clearer insight into how ISSPA works. 
In the meantime, you will also understand ISSPA's program logic and how its components relate to each other. 

The tutorial includes the following parts:

- `ISSPA Python package`_

    - :ref:`Creation method <pycm>`

    - :ref:`Usage examples <pyue>`

- `ISSPA C++ library`_

    - :ref:`Creation method <ccm>`

    - :ref:`Usage examples <cue>`


ISSPA Python Package
--------------------

First, you need to understand how to create a Python package in ROS and how other ROS packages import it. 
Second, you will be introduced to the basic usage of Python package with code samples.

.. _`pycm`:

Creation Method
~~~~~~~~~~~~~~~

Here's an example of creating a ROS package named ``isspa``, which contains a Python package also named ``isspa``.
Create a ``setup.py`` file in the root directory of the ``isspa`` ROS package, i.e., the directory containing ``CMakeLists.txt``:

.. code-block:: bash

    mkdir -p ~/ISSPA/src
    cd ~/ISSPA/src
    catkin_create_pkg isspa rospy roscpp std_msgs
    cd isspa
    touch setup.py
    touch scripts/isspa/__init__.py
    touch scripts/isspa/logging_output.py
    chomd +x scripts/isspa/logging_output.py

The content of the ``setup.py`` file is the following:

.. code-block:: python

    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup


    d = generate_distutils_setup(
        packages=['isspa'],
        package_dir={'': 'scripts'},
        version='0.1.0'
    )

    setup(**d)

Only two lines are important here, the rest are the same every time:

.. code-block:: python

    packages=['isspa'],
    package_dir={'': 'scripts'},

The first line indicates the name of the package, and the second line indicates the directory where the package is located.

.. note::

    The version should be consistent with the version of the ROS package in the ``package.xml`` file.

The content of the ``logging_output.py`` file is like the following:

.. code-block:: python

    #!/usr/bin/env python
    import rospy

    def logger(msg):
        rospy.loginfo("Welcome to ISSPA. " + msg)


After that, you also need to change the contents of ``CMakeLists.txt``:

.. code-block:: cmake

    # uncomment these two lines 

    catkin_python_setup()

    catkin_package()


Finally, you need to build the package:

.. code-block:: bash

    cd ~/ISSPA
    catkin_make
    source devel/setup.bash


If all goes well, you can try running the program in another package. A simple test is as follows:

.. code-block:: bash

    # Open a new terminal and run
    roscore
    # after having sourced devel/setup.bash

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

Then, we run the following command to make the script executable:

.. code-block:: bash

    # In another terminal, after having sourced devel/setup.bash
    rosrun test_import_python_package test_import_python_package.py

If everything is working as expected, the following messages will be output:

.. code-block:: bash

    [INFO] [1632938655.753961]: Welcome to ISSPA. This is a test message.
    [INFO] [1632938655.758263]: Welcome to ISSPA! This is a message from test_import_python_package.py


.. _`pyue`:

Usage Examples
~~~~~~~~~~~~~~

*Needs to be replenished.*


ISSPA C++ Library
-----------------

The ROS library configuration for C++ is more complex than for Python, so let's learn how to create it!

.. _`ccm`:

Creation Method
~~~~~~~~~~~~~~~

Here's an example of creating a C++ library named ``isspa``, which also provides a header file named ``util.h``.

.. code-block:: bash

    mkdir -p ~/ISSPA/src
    cd ~/ISSPA/src
    catkin_create_pkg isspa rospy roscpp std_msgs
    cd isspa
    mkdir include
    touch include/isspa/util.h

The content of the ``util.h`` file is:

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

    Don't misspell ``#ifndef`` as ``#ifdef``, otherwise the contents of the header file will not be found during compilation.

Then create ``util.cpp`` in the ``~/ISSPA/src/isspa/src`` directory with the following content:

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

After that, you also need to change the content of ``CMakeLists.txt`` within the ``isspa`` package:

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

After that, you also need to change the content of ``CMakeLists.txt`` within the ``test_import_roscpp_library`` package:

.. code-block:: cmake

    # modify this part

    find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    isspa
    )

    # and these lines

    add_executable(test_import_roscpp_library src/main.cpp)
    target_link_libraries(test_import_roscpp_library ${catkin_LIBRARIES})

and add a ``<depend>isspa</depend>`` tag inside the ``package.xml``.

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

The following messages are expected to be output:

.. code-block:: bash

    [Welcome to ISSPA util] Hello from util.cpp
    [INFO] [1632938655.757961]: Hello from util.cpp


.. _`cue`:

Usage Examples
~~~~~~~~~~~~~~

*Needs to be replenished.*


Reference
---------

- `[ROS] Include a Cpp header from another package <https://roboticsbackend.com/ros-include-cpp-header-from-another-package/>`_

- `[ROS] How To Import a Python Module From Another Package <https://roboticsbackend.com/ros-import-python-module-from-another-package/>`_





