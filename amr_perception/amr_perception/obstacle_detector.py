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
        
        front_ranges = []
        left_ranges = []
        right_ranges = []

        for index, distance in enumerate(msg.ranges):

            if not math.isfinite(distance):
                continue

            if not (
                msg.range_min
                <= distance
                <= msg.range_max
            ):
                continue

            angle = (
                msg.angle_min
                + index * msg.angle_increment
            )

            angle = math.atan2(
                math.sin(angle),
                math.cos(angle)
            )

            angle_deg = math.degrees(angle)

            # FRONT: -30° to +30°
            if -30.0 <= angle_deg <= 30.0:
                front_ranges.append(distance)

            # LEFT: +30° to +90°
            elif 30.0 < angle_deg <= 90.0:
                left_ranges.append(distance)

            # RIGHT: -90° to -30°
            elif -90.0 <= angle_deg < -30.0:
                right_ranges.append(distance)

        front_distance = (
            min(front_ranges)
            if front_ranges
            else float('inf')
        )

        left_distance = (
            min(left_ranges)
            if left_ranges
            else float('inf')
        )

        right_distance = (
            min(right_ranges)
            if right_ranges
            else float('inf')
        )

        self.get_logger().info(
            f'Front: {front_distance:.2f} m | '
            f'Left: {left_distance:.2f} m | '
            f'Right: {right_distance:.2f} m'
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