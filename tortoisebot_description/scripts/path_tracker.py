#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
import math

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')
        
       
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        
       
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
      
        self.current_pose = None
        self.path_history = Path()
        self.path_history.header.frame_id = "odom"
        
        
        self.waypoints = [
            (2.0, 0.0),  
            (2.0, 2.0),  
            (0.0, 2.0),   
            (0.0, 0.0),   
        ]
        self.current_waypoint_idx = 0
        self.goal_tolerance = 0.3
        
      
        self.max_linear_speed = 0.5
        self.max_angular_speed = 1.0
        
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Path Tracker Node Started')
        
    def odom_callback(self, msg):
        """Update current pose and path history"""
        self.current_pose = msg.pose.pose
        
      
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = self.current_pose
        self.path_history.poses.append(pose_stamped)
        
        
        if len(self.path_history.poses) > 1000:
            self.path_history.poses.pop(0)
        
        
        self.path_history.header.stamp = self.get_clock().now().to_msg()
        
       
        self.path_pub.publish(self.path_history)
        self.publish_markers()
    
    def control_loop(self):
        """Main control loop for waypoint following"""
        if self.current_pose is None:
            return
            
        if self.current_waypoint_idx >= len(self.waypoints):
           
            self.stop_robot()
            return
            
        
        goal_x, goal_y = self.waypoints[self.current_waypoint_idx]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        
        distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        
        if distance < self.goal_tolerance:
          
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx-1}')
            return
            
        
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        
      
        current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        
        angular_error = self.normalize_angle(angle_to_goal - current_yaw)
        
   
        cmd = Twist()
    
        cmd.angular.z = max(-self.max_angular_speed, 
                           min(self.max_angular_speed, 2.0 * angular_error))
        
        if abs(angular_error) < 0.5:  
            cmd.linear.x = max(0.0, min(self.max_linear_speed, distance))
        
        self.cmd_vel_pub.publish(cmd)
        
       
        self.publish_goal_marker(goal_x, goal_y)
    
    def publish_markers(self):
        """Publish visualization markers for the path"""
        marker_array = MarkerArray()
        
        if len(self.path_history.poses) < 2:
            return
            
        path_marker = Marker()
        path_marker.header.frame_id = "odom"
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = "robot_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        path_marker.scale.x = 0.02
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0
        
       
        for pose_stamped in self.path_history.poses:
            path_marker.points.append(pose_stamped.pose.position)
            
        marker_array.markers.append(path_marker)
        self.marker_pub.publish(marker_array)
    
    def publish_goal_marker(self, x, y):
        """Publish goal marker"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        
        self.goal_marker_pub.publish(marker)
    
    def quaternion_to_yaw(self, quaternion):
        """Convert quaternion to yaw angle"""
        import math
        siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop_robot(self):
        """Stop the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    path_tracker = PathTracker()
    rclpy.spin(path_tracker)
    path_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()