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

## Installation and Setup

1. Create a ROS 2 workspace (if you don’t have one):

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. Clone the repository:

   ```bash
   git clone https://github.com/your_username/tortoisebot_lidar_tracker.git
   ```

3. Install Python dependencies:

   ```bash
   pip install -r tortoisebot_lidar_tracker/requirements.txt
   ```

4. Build the workspace:

   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

---

## Usage

1. Launch your TortoiseBot Gazebo simulation with LIDAR enabled:

   ```bash
   ros2 launch tortoisebot_gazebo tortoisebot_world.launch.py
   ```

2. Run the closest object detector node:

   ```bash
   ros2 run tortoisebot_lidar_tracker closest_object_detector
   ```

3. The node will:

   * Subscribe to `/scan` (`sensor_msgs/msg/LaserScan`)
   * Focus on ±15° in front of the robot
   * Filter out invalid readings (`inf`, `nan`, `0`)
   * Apply median/min filtering to reduce spikes
   * Publish filtered distance to `/closest_object_distance` (`std_msgs/msg/Float32`)

4. You can echo the published distance:

   ```bash
   ros2 topic echo /closest_object_distance
   ```

---

## Simulation Notes

* Ensure LIDAR plugin is enabled in the Gazebo TortoiseBot URDF.
* Make sure the simulation publishes the `/scan` topic.
* Adjust the filtering method (median, minimum, moving average) in the node as needed.

---

## Example Output

```
Closest object distance: 0.85 m
Closest object distance: 0.82 m
Closest object distance: 0.80 m
...
```

---

## License

MIT License

```



