# ros2_jazzy_lidar_object_detector
TortoiseBot LIDAR Tracker is a ROS 2 Python package designed for real-time obstacle detection and proximity monitoring in Gazebo-based TortoiseBot simulations. The package subscribes to the `/scan` topic published by a LIDAR sensor, processes the laser scan data to detect the closest object directly in front of the robot within a configurable field of view (±15° by default), and publishes the filtered distance to `/closest_object_distance`.  

The node implements robust data filtering by ignoring invalid range readings (`inf`, `nan`, or `0`) and applying spike reduction techniques, such as median or minimum value filtering, to ensure reliable distance measurements. This package is ideal for developers and researchers working on reactive navigation, obstacle avoidance, and autonomous robot behaviors in simulated environments.  

Key features include:  
- Real-time detection of closest objects in front of the robot  
- Configurable field-of-view filtering  
- Spike reduction and noise filtering for stable readings  
- Integration with Gazebo simulations and ROS 2 ecosystem  
- Published output suitable for downstream navigation and control nodes
    
click [here](https://youtu.be/Oc131PJi4ho) to see the output video.


---

## System Requirements

- **OS**: Ubuntu 24.04  
- **ROS 2**: Jazzy Jalisco  
- **Simulator**: Gazebo Harmonic  
- **Python**: 3.10+  
- **Build Tool**: colcon  


## Installation & Setup

```bash
# Clone the repository
git clone https://github.com/sivapriya083/ros2_jazzy_lidar_object_detector.git

# Install Python dependencies
pip install -r ros2_jazzy_lidar_object_detector/requirements.txt

# Build the workspace
cd ~/ros2_ws
colcon build

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Source your workspace
source install/setup.bash
```

---

## Usage

1. Launch the TortoiseBot Gazebo simulation with LIDAR enabled:

   ```bash
   ros2 launch tortoisebot_gazebo tortoisebot_world.launch.py
   ```
    ![](https://github.com/Sivapriya083/ros2_jazzy_lidar_object_detector/blob/main/lidar.png?raw=true)
    To enable LIDAR Visualization:

      *Click on the three dots on the upper right corner og your gazebo window.
      *Search for Visualize LIdar from the menu and click on it.
      *Refresh the list of topics and choose the "/scan" topic.Ensure that display LIdar Visualization is checked.

3. Run the closest object detector node:
    In a new terminal after sourcing run:

   ```bash
   ros2 run ros2_jazzy_lidar_object_detector closest_object_detector
   ```

4. Control the robot
   
---

### **Default Keyboard Controls (`teleop_twist_keyboard`)**

* **Movement:**

  * `i` → move forward
  * `,` → move backward
  * `j` → turn left
  * `l` → turn right
  * `u` → move forward-left (diagonal)
  * `o` → move forward-right
  * `m` → move backward-left
  * `.` → move backward-right

* **Stop:**

  * `k` → stop

* **Speed Adjustments:**

  * `q` → increase linear speed
  * `z` → decrease linear speed
  * `w` → increase angular speed
  * `x` → decrease angular speed

* **Exit:**

  * `CTRL+C` → quit the node

---


5. The node will:

   * Subscribe to `/scan` (`sensor_msgs/msg/LaserScan`)
   * Focus on ±15° in front of the robot
   * Filter out invalid readings (`inf`, `nan`, `0`)
   * Apply median/min filtering to reduce spikes
   * Publish filtered distance to `/closest_object_distance` (`std_msgs/msg/Float32`)

6. To visualize the published distance:

   ```bash
   ros2 topic echo /closest_object_distance
   ```

---

## Simulation Notes

* Ensure the LIDAR plugin is enabled in the Gazebo TortoiseBot URDF.
* Confirm that the `/scan` topic is publishing.
* Filtering method (median, minimum, moving average) can be adjusted in the node code.

---

## Example Output

```
Closest object distance: 0.85 m
Closest object distance: 0.82 m
Closest object distance: 0.80 m
```

---

## License

This project is licensed under the **MIT License**.





