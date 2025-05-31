#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import numpy as np
import struct
import math

class PointCloudToLaserScanNode(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan_node')
        
        # Parameters
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/scan_2d')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('min_height', -0.5)
        self.declare_parameter('max_height', 2.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi/180.0)  # 1 degree
        self.declare_parameter('scan_time', 0.1)
        self.declare_parameter('range_min', 0.1)
        self.declare_parameter('range_max', 30.0)
        
        # Get parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.min_height = self.get_parameter('min_height').value
        self.max_height = self.get_parameter('max_height').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.scan_time = self.get_parameter('scan_time').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        
        # Calculate number of laser scan rays
        self.num_ranges = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Publisher and subscriber
        self.laser_pub = self.create_publisher(LaserScan, self.output_topic, 10)
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, 
            self.input_topic, 
            self.pointcloud_callback, 
            10
        )
        
        self.get_logger().info(f'PointCloud to LaserScan converter started')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Output topic: {self.output_topic}')
        self.get_logger().info(f'Height range: {self.min_height} to {self.max_height}')
        self.get_logger().info(f'Angle range: {math.degrees(self.angle_min)} to {math.degrees(self.angle_max)} degrees')
        
    def pointcloud_callback(self, msg):
        """Convert PointCloud2 to LaserScan"""
        try:
            # Create LaserScan message
            scan = LaserScan()
            scan.header = msg.header
            scan.header.frame_id = self.frame_id
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.time_increment = 0.0
            scan.scan_time = self.scan_time
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            
            # Initialize ranges array
            ranges = [float('inf')] * self.num_ranges
            
            # Parse point cloud data
            points = self.parse_pointcloud2(msg)
            
            if len(points) == 0:
                self.get_logger().warn('No points found in point cloud')
                return
                
            # Filter points by height and convert to laser scan
            valid_points = 0
            for point in points:
                x, y, z = point[:3]
                
                # Filter by height
                if z < self.min_height or z > self.max_height:
                    continue
                    
                valid_points += 1
                
                # Calculate range and angle
                range_val = math.sqrt(x*x + y*y)
                
                # Skip points too close or too far
                if range_val < self.range_min or range_val > self.range_max:
                    continue
                    
                angle = math.atan2(y, x)
                
                # Skip points outside angle range
                if angle < self.angle_min or angle > self.angle_max:
                    continue
                    
                # Calculate index in ranges array
                index = int((angle - self.angle_min) / self.angle_increment)
                if 0 <= index < self.num_ranges:
                    # Keep the closest point for each angle
                    if range_val < ranges[index]:
                        ranges[index] = range_val
            
            # Replace inf with max range
            for i in range(len(ranges)):
                if math.isinf(ranges[i]):
                    ranges[i] = self.range_max
                    
            scan.ranges = ranges
            scan.intensities = []  # Not using intensities
            
            # Publish the scan
            self.laser_pub.publish(scan)
            
            self.get_logger().debug(f'Converted {valid_points} points to laser scan with {len(ranges)} rays')
            
        except Exception as e:
            self.get_logger().error(f'Error converting point cloud: {str(e)}')
    
    def parse_pointcloud2(self, msg):
        """Parse PointCloud2 message and extract XYZ points"""
        points = []
        
        # Get point step (bytes per point)
        point_step = msg.point_step
        
        # Find XYZ field offsets
        x_offset = None
        y_offset = None
        z_offset = None
        
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
                
        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().error('Could not find XYZ fields in point cloud')
            return points
            
        # Extract points
        for i in range(0, len(msg.data), point_step):
            if i + point_step > len(msg.data):
                break
                
            try:
                # Extract XYZ coordinates (assuming float32)
                x = struct.unpack_from('f', msg.data, i + x_offset)[0]
                y = struct.unpack_from('f', msg.data, i + y_offset)[0]
                z = struct.unpack_from('f', msg.data, i + z_offset)[0]
                
                # Skip invalid points
                if math.isnan(x) or math.isnan(y) or math.isnan(z):
                    continue
                    
                points.append([x, y, z])
                
            except (struct.error, IndexError):
                continue
                
        return points

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScanNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
