ISSPA Documentation
===================================


Welcome to `ISSPA <https://github.com/chenhengwei1999/ISSPA>`_ documentation!
-------------------------------------------------------------------------------

.. toctree::
   :maxdepth: 1
   :caption: HOME 

.. toctree::
   :maxdepth: 1
   :caption: TUTORIALS
   :hidden:

   /tutorial/quick_start
   /tutorial/pavs_quick_start

.. toctree::
   :maxdepth: 3
   :caption: ISSPA COMPONENTS
   :hidden:

   /components/chassis
   /components/sensors
   /components/mapping
   /components/navigation
   /components/detection
   /components/segmentation

.. toctree::
   :maxdepth: 2
   :caption: CONTRIBUTIND TO ISSPA
   :hidden:

   /contributing_to_isspa/mapping_plugin.rst
   /contributing_to_isspa/navigation_plugin.rst
   /contributing_to_isspa/object_detection_plugin.rst

.. toctree::
   :maxdepth: 2
   :caption: ABOUT US
   :hidden:

   /about_us/team.rst


.. meta::
   :description lang=en: Automate building, version=0.1, and hosting of your technical documentation continuously on Read the Docs.

.. Adds a hidden link for the purpose of validating Read the Docs' Mastodon profile
.. raw:: html

   <a style="display: none;" rel="me" href="https://fosstodon.org/@readthedocs">Mastodon</a>

**ISSPA (Intelligent Self-driving System empowering Physical Agents)** is an experimental platform for 
self-driving physical agents, which was originally designed to facilitate researchers by creating
baseline solutions that are easy to use, algorithmically verifiable, and modularly extensible. And it was 
built under the leadership of the `TIS Lab <https://tis.ios.ac.cn/>`_, of which the team members 
are listed in :doc:`about_us/team`.


.. note::
   The ISSPA project is a branch of the `ISS <https://tis.ios.ac.cn/iss/>`_ project.

ISSPA has the following features:

- Some baseline solutions for autonomous driving.

- Cloud platform that supports the remote use of physical vehicles.


.. figure:: imgs/isspa_overview.jpg
   :alt: ISSPA Overview
   :align: center

   **Overview of ISSPA**


TUTORIALS
---------
Here are some guidelines for you to get started:

- :doc:`/tutorial/quick_start` - Understand the operating environment and how programs are launched.


ISSPA COMPONENTS
----------------
Introduction to the core elements of ISSPA.

- :doc:`/components/chassis` - Descibes what ISSPA's vehicle chassis is, and how to get vehicle's state.
  
- :doc:`/components/sensors` - Describe the sensors used by ISSPA, e.g., camera, lidar.
  
- :doc:`/components/mapping` - The dominant algorithm for mapping, the baseline algorithm, is introduced.
  
- :doc:`/components/navigation` - Navigation frameworks and classical planning algorithms are presented.
  
- :doc:`/components/detection` - Introduction to target detection tasks, and common baseline algorithms such as yolov5.

- :doc:`/components/segmentation` - Introduction to target segmentation tasks, and common baseline algorithms such as LSS.


CONTRIBUTIND TO ISSPA
------------------------
Guide you on how to add plugins to ISSPA, if you have a better solution and are willing to contribute to this project, 
please do not hesitate to Pull&Request on `Github: ISSPA <https://github.com/chenhengwei1999/ISSPA/>`_.

- :doc:`/contributing_to_isspa/mapping_plugin` - Extended guidance on SLAM algorithms is available here.
  
- :doc:`/contributing_to_isspa/navigation_plugin` - Extended guidance on navigation algorithms is available here.
  
- :doc:`/contributing_to_isspa/object_detection_plugin` - Extended guidance on object detection algorithms is available here.


ABOUT US
--------

- :doc:`/about_us/team` - Introduce our team.
