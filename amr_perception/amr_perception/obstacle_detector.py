#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ObstacleDetector(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.scan_subscriber_ = self.create_subscription(msg_type=LaserScan, topic='/scan', callback=self.scan_callback, qos_profile=10)
    
    def scan_callback(self, msg: LaserScan):
        """Process incoming LaserScan messages to detect obstacles."""
        
        valid_ranges = [
            r for r in msg.ranges 
            if not math.isfinite(r) 
            and msg.range_min <= r <= msg.range_max
            ]
        
        if not valid_ranges:
            self.get_logger().warn(
                'No valid LiDAR measurements received.'
            )
            return
        
        minimum_distance = min(valid_ranges)
        
        self.get_logger().info(
            f'Nearest obstacle: {minimum_distance:.2f} meters'
        )
        

def main(args=None):

    rclpy.init(args=args)

    node = ObstacleDetector(name="obstacle_detector")

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()