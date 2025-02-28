## radar_stationRM2024
Solution for the radar station of HKU Astar &amp; Robomaster for Season 2023/2024

## Package Discription
├── radar_stationRM2024   
│   ├── custom_msgs: declared all custom ROS message types, as well as yaml configuration files and launch files   
│   ├── detection_pkg: package for detecting robots using YOLO from camera images   
│   ├── distance_pkg: package for measuring distance of detected robots using LiDAR and sensor fusion   
│   ├── uimap_pkg: package for mapping depth points onto the map based on the official competition coordinates   
│   ├── gui_pkg: package for the graphical user interface, used for testing and calibration purposes.


# How to run

```
roslaunch custom_msgs launcher.launch
```

