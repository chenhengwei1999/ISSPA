# Quick Start of PAVS (Physical Agent Vehicle Small)

## Start the Vehicle Chassis

Firstly, launch chassis and sensors driver of the vehicle.

```bash
roslaunch pavs_bringup pavs_chassis_and_sensors.launch
```

## Start the SLAM Program

```bash
roslaunch mapping_baselines pavs_map.launch
```

When the map is created, you can execute `map.sh` under the `~/pa_ws/src/ISSPA/src/isspa_mapping/mapping_baselines/scripts` directory to save your map.

## Start the Navigation Program

```bash
roslaunch navigation_stack pavs_navigation.launch
```




