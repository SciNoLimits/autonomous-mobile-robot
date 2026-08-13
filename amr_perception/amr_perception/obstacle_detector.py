#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException


class ObstacleDetector(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.scan_subscriber_ = self.create_subscription(msg_type=LaserScan, topic='/scan', callback=self.scan_callback, qos_profile=10)
    
    def scan_callback(self, msg):

        self.get_logger().info(
            f'Received LaserScan: '
            f'{len(msg.ranges)} ranges, '
            f'range_min={msg.range_min:.2f}, '
            f'range_max={msg.range_max:.2f}'
        )

        valid_ranges = [
            r for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        self.get_logger().info(
            f'Valid ranges: {len(valid_ranges)}'
        )

        if not valid_ranges:
            self.get_logger().warn(
                'No valid LiDAR measurements received.'
            )
            return

        minimum_distance = min(valid_ranges)

        self.get_logger().info(
            f'Nearest obstacle: {minimum_distance:.2f} m'
        )
        

def main(args=None):

    rclpy.init(args=args)
    node = ObstacleDetector(name="obstacle_detector")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()