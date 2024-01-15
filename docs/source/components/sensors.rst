**Sensors**
======================

.. meta::
   :description lang=en: Automate building, version=0.1, and hosting of your technical documentation continuously on Read the Docs.

.. raw:: html

    <a style="display: none;" rel="me" href="https://fosstodon.org/@readthedocs">Mastodon</a>

**Preface**
-----------

The proper deployment and standardized use of sensors are crucial for the stable operation of autonomous 
vehicles, as the quality of their data significantly impacts the safety of vehicle navigation. 
In the field of autonomous driving, the array of sensors typically includes industrial cameras, LiDAR, 
mm-wave radar, GPS, IMU, each with distinct data formats.

**ISSPA's configuration**
-------------------------

ISSPA's vehicles can be equipped with single/multi-line LiDAR, mono/depth cameras, IMU, 
providing researchers with a diverse range of options to meet their specific needs. 
Brief introductions to different sensors are as follows: 

- LiDAR: LiDAR utilizes laser beams to get distances by measuring the reflection time. 
  Widely used in autonomous driving, LiDAR contributes to real-time mapping, object detection. 
  Below is a frame of point cloud from one campus using velodyne's 16-line LiDAR.

  .. figure:: ../imgs/lidar.png
    :alt: LiDAR example
    :align: center
    :scale: 50%

    **Sample LiDAR point cloud data collected at one campus**

- Depth Camera: Depth cameras based on Time-of-Flight (ToF) technology use the principle of measuring the flight 
  time of light pulses to determine the distance between objects and the camera. ToF depth cameras offer advantages 
  of speed, accuracy, and adaptability to various lighting conditions. They find widespread applications in computer 
  vision, 3D scanning, virtual reality, and augmented reality.

  .. figure:: ../imgs/astro_pro_plus.jpeg
    :alt: Camera example
    :align: center
    :scale: 50%

    **Data visualization of different modalities of depth cameras in one laboratory**
  

- IMU (Inertial Measurement Unit): IMU provides information about a vehicle's attitude and motion 
  state by measuring acceleration and angular velocity. In autonomous driving systems, 
  IMU is commonly used to enhance navigation accuracy and stability.

  .. figure:: ../imgs/imu.png
    :alt: IMU example
    :align: center
    :scale: 50%

    `IMU example <https://www.semanticscholar.org/paper/Gait-dynamics-sensing-using-IMU-sensor-array-system-Kardos%CC%8C-Balog/55e6ad65ed6249f6a50d83cca1188b688febadc1/figure/0>`_

Through the thoughtful selection and integration of these sensors, ISSPA aims to offer a flexible and diverse range of choices 
to meet various research requirements in different scenarios.