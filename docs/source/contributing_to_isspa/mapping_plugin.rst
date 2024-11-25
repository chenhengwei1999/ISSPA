**Mapping Plugin**
==================

**Current Listings**
--------------------

As of now, ISSPA has integrated the following baseline algorithms:

+---------------------+----------------------+
|      2D SLAM        |        3D SLAM       |
+=====================+======================+
|      GMapping       |        A-LOAM        |
+---------------------+----------------------+
|       Hector        |        LIO-SAM       |
+---------------------+----------------------+
|        Karto        |       Lego-LOAM      |
+---------------------+----------------------+
|    Cartographer     |       RTAB-Map       |
+---------------------+----------------------+


.. note::
    A-LOAM: Advanced Lidar Odometry and Mapping; 
    LIO-SAM: Lidar Inertial Odometry via Smoothing and Mapping;
    RTAB-Map: Real-Time Appearance-Based Mapping; 


**Adding Plugins**
------------------

The structure of the document has been designed in ISSPA and you can contribute to us by following the tutorials below.
Just use `isspa_mapping_plugin` as the name of your algorithm:

.. code-block:: bash
    
    mkdir -p ~/pa_ws/src && cd ~/pa_ws/src
    git clone https://github.com/chenhengwei1999/ISSPA.git
    cd ISSPA/src/isspa_mapping
    catkin_create_pkg isspa_mapping_plugin rospy roscpp std_msgs

Then, you can add your own algorithm in the ``isspa_mapping_plugin`` package.

Once you have added it, you can compile the feature package with the help of the ``catkin_make`` tool to see if it is 
available and if it runs smoothly. If it runs successfully, you can open a Github Pull Request to submit your merge request.
