#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math

class ClosestObjectDetector(Node):
    def __init__(self):
        super().__init__('closest_object_detector')
        
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.distance_pub = self.create_publisher(Float32, '/closest_object_distance', 10)
        
       
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
      
        self.front_angle = math.radians(15)  
        self.reading_history = []            
        self.max_history_size = 5
       
        self.should_stop = False
        self.current_distance = float('inf')
        self.last_safe_cmd = Twist()
        
        self.get_logger().info('Closest Object Detector with Stop Function started')

    def cmd_vel_callback(self, msg):
       
        if self.should_stop:
            
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)
        else:
            
            self.last_safe_cmd = msg
          

    def scan_callback(self, msg):
       
        front_ranges = []
        
        for i, range_val in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            
            if abs(angle) <= self.front_angle:
                if (not math.isinf(range_val) and 
                    not math.isnan(range_val) and 
                    range_val > 0 and 
                    msg.range_min <= range_val <= msg.range_max):
                    front_ranges.append(range_val)

        if front_ranges:
         
            closest_distance = min(front_ranges)
            filtered_distance = self.apply_filters(closest_distance)
            
           
            distance_msg = Float32()
            distance_msg.data = filtered_distance
            self.distance_pub.publish(distance_msg)
            
          
            self.current_distance = filtered_distance
            was_stopping = self.should_stop
            self.should_stop = filtered_distance <= 0.7
            
         
            if self.should_stop and not was_stopping:
                stop_cmd = Twist()
                self.cmd_vel_pub.publish(stop_cmd)
            
          
            
            self.get_logger().info(f'Closest object distance: {filtered_distance:.2f}m')

    def apply_filters(self, new_distance):
     
        self.add_reading(new_distance)
        
        if len(self.reading_history) >= 3:  
            median_result = self.get_median(self.reading_history)
            average_result = self.get_moving_average(self.reading_history)
            minimum_result = min(self.reading_history)
            
            return median_result
        else:
            return new_distance

    def add_reading(self, new_reading):
     
        self.reading_history.append(new_reading)
        
        if len(self.reading_history) > self.max_history_size:
            self.reading_history.pop(0)

    def get_median(self, readings):
      
        sorted_readings = sorted(readings)
        middle_index = len(sorted_readings) // 2
        return sorted_readings[middle_index]

    def get_moving_average(self, readings):
       
        total = sum(readings)
        count = len(readings)
        return total / count

def main(args=None):
    rclpy.init(args=args)
    node = ClosestObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()