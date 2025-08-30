# ros2_jazzy_lidar_object_detector
TortoiseBot LIDAR Tracker is a ROS 2 Python package designed for real-time obstacle detection and proximity monitoring in Gazebo-based TortoiseBot simulations. The package subscribes to the `/scan` topic published by a LIDAR sensor, processes the laser scan data to detect the closest object directly in front of the robot within a configurable field of view (±15° by default), and publishes the filtered distance to `/closest_object_distance`.  

The node implements robust data filtering by ignoring invalid range readings (`inf`, `nan`, or `0`) and applying spike reduction techniques, such as median or minimum value filtering, to ensure reliable distance measurements. This package is ideal for developers and researchers working on reactive navigation, obstacle avoidance, and autonomous robot behaviors in simulated environments.  

Key features include:  
- Real-time detection of closest objects in front of the robot  
- Configurable field-of-view filtering  
- Spike reduction and noise filtering for stable readings  
- Integration with Gazebo simulations and ROS 2 ecosystem  
- Published output suitable for downstream navigation and control nodes
    
click [here](https://youtu.be/Oc131PJi4ho) to see the output.


---

## System Requirements
- Ubuntu 24.04  
- ROS 2 Jazzy  
- Gazebo Harmonic


---

## Prerequisites

1. ROS 2 workspace:  
   ```bash
   mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
````

2.Source ROS 2 environment:

   ```bash
      source /opt/ros/humble/setup.bash
   ````
