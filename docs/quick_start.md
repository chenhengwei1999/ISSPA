# Quick Start of PAVS (Physical Agent Vehicle Small)

## Start the Vehicle Chassis

Firstly, launch chassis and sensors driver of the vehicle.

```bash
roslaunch vehicle_bringup vehicle_chassis_and_sensors.launch
```

## Start the SLAM Program

```bash
roslaunch vehicle_mapping pavs_map.launch
```

When the map is created, you can execute `map.sh` under the `~/chw_space/pavs_ws/src/ISSPA/src/vehicle_mapping/scripts` directory to save your map.

## Start the Navigation Program

```bash
roslaunch vehicle_navigation pavs_navigation.launch
```




