**Detection**
======================


.. meta::
   :description lang=en: Automate building, version=0.1, and hosting of your technical documentation continuously on Read the Docs.

.. raw:: html

    <a style="display: none;" rel="me" href="https://fosstodon.org/@readthedocs">Mastodon</a>

Currently widely used sensing sensors in autonomous driving include LiDAR, cameras, radar, and event-based cameras (e.g., DVS).
Based on such sensors, we can implement tasks such as object detection algorithms, instance segmentation algorithms, 
and drivable area detection. In ISSPA, we categorize detection tasks based on the type of sensor:

- Camera based

- LiDAR based

- Fusion based


**Camera Based**
-------------------

.. meta::
   :description lang=en: Automate building, version=0.1, and hosting of your technical documentation continuously on Read the Docs.

.. raw:: html

    <a style="display: none;" rel="me" href="https://fosstodon.org/@readthedocs">Mastodon</a>

Traffic light Recognition
~~~~~~~~~~~~~~~~~~~~~~~~~

In the domain of autonomous driving, the recognition of traffic signal lights holds pivotal importance. This capability involves 
the vehicle system identifying and comprehending the status of traffic signals, including red, green, and yellow lights.

In ISSPA, we trained a classifier based on yolov5 that can recognize the color of traffic lights with the following results:

.. figure:: ../imgs/yolov5_traffic_light_green.jpg
   :alt: Yolov5 Traffic light Green
   :align: center
   :scale: 50%

   **Traffic light recognition results based on yolov5 (green)**

.. figure:: ../imgs/yolov5_traffic_light_yellow.jpg
   :alt: Yolov5 Traffic light Yellow
   :align: center
   :scale: 50%

   **Traffic light recognition results based on yolov5 (yellow)**


**Lidar Based**
-------------------

Lidar plays a critical role in autonomous driving due to its high precision, real-time scanning capability, 360-degree perception, 
and adaptability to diverse environmental conditions. As a key sensor, Lidar contributes significantly to the safe and reliable 
operation of autonomous vehicles.

.. note::

    The PointPillars algorithm has been tested in ISSPA and will be synchronized to the code repository as soon as possible.