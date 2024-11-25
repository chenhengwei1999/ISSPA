**Navigation Plugin**
=====================

**Current Listings**
--------------------

As of now, ISSPA has integrated the following baseline algorithms:


+----------------------+----------------------+
|     Global Planner   |     Local Planner    |
+======================+======================+
|          A*          |         TEB          |
+----------------------+----------------------+
|          D*          |         DWA          |
+----------------------+----------------------+
|          PSO         |         ...          |
+----------------------+----------------------+


.. note::
    PSO: Particle Swarm Optimization; 
    TEB: Timed Elastic Band; 
    DWA: Dynamic Window Approach; 


**Adding Plugins**
------------------

The structure of the document has been designed in ISSPA and you can contribute to us by following the tutorials below.
Just use `isspa_navigation_plugin` as the name of your algorithm:

.. code-block:: bash
    
    mkdir -p ~/pa_ws/src && cd ~/pa_ws/src
    git clone https://github.com/chenhengwei1999/ISSPA.git
    cd ISSPA/src/isspa_navigation
    catkin_create_pkg isspa_navigation_plugin rospy roscpp std_msgs

Then, you can add your own algorithm in the ``isspa_navigation_plugin`` package.

Once you have added it, you can compile the feature package with the help of the ``catkin_make`` tool to see if it is 
available and if it runs smoothly. If it runs successfully, you can open a Github Pull Request to submit your merge request.