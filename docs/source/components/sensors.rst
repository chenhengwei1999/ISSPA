**Sensors**
===========


**Preface**
-----------

The proper deployment and standardized use of sensors are crucial for the stable operation of autonomous 
vehicles, as the quality of their data significantly impacts the safety of vehicle navigation. 
In the field of autonomous driving, the array of sensors typically includes industrial cameras, LiDAR, 
mm-wave radar, GPS, and IMU, each with distinct data formats.

**ISSPA's configuration**
-------------------------

ISSPA's vehicles can be equipped with single/multi-line LiDAR, mono/depth cameras, and IMU, 
providing researchers with a diverse range of options to meet their specific needs. 
Brief introductions to different sensors are as follows: 

- **LiDAR**: a LiDAR utilizes laser beams to get distances by measuring the reflection time. 
  Widely used in autonomous driving, LiDAR contributes to real-time mapping and object detection. 
  Below is a frame of point cloud from a campus environment using velodyne's 16-line LiDAR.

  .. figure:: ../imgs/lidar.png
    :alt: LiDAR example
    :align: center
    :scale: 50%

    **Sample LiDAR point cloud data collected at a campus environment**

- **Depth Camera**: depth cameras based on Time-of-Flight (ToF) technology use the principle of measuring the flight 
  time of light pulses to determine the distance between objects and the camera. ToF depth cameras offer advantages 
  of speed, accuracy, and adaptability to various lighting conditions. They find widespread applications in computer 
  vision, 3D scanning, virtual reality, and augmented reality.

  .. figure:: ../imgs/astro_pro_plus.jpeg
    :alt: Camera example
    :align: center
    :scale: 50%

    **Data visualization of different modalities of depth cameras in a laboratory environment**
  

- **IMU** (Inertial Measurement Unit): an IMU provides information about a vehicle's attitude and motion 
  state by measuring acceleration and angular velocity. In autonomous driving systems, 
  an IMU is commonly used to enhance navigation accuracy and stability.

  .. figure:: ../imgs/imu.png
    :alt: IMU example
    :align: center
    :scale: 50%

    `IMU example <https://www.semanticscholar.org/paper/Gait-dynamics-sensing-using-IMU-sensor-array-system-Kardos%CC%8C-Balog/55e6ad65ed6249f6a50d83cca1188b688febadc1/figure/0>`_

Through the thoughtful selection and integration of these sensors, ISSPA aims to offer a flexible and diverse range of choices 
to meet various research requirements in different scenarios.