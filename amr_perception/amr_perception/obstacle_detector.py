#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.executors import ExternalShutdownException
from amr_interfaces.msg import ObstacleStatus # type: ignore

import tf2_geometry_msgs
from tf2_ros import TransformListener, Buffer
# from tf_transformations import euler_from_quaternion
from rclpy.time import Time
from geometry_msgs.msg import PointStamped


class ObstacleDetector(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.declare_parameter('obstacle_threshold', 0.5)  # meters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        
        self.scan_subscriber_ = self.create_subscription(msg_type=LaserScan, 
                                                         topic='/scan', 
                                                         callback=self.scan_callback, 
                                                         qos_profile=10
                                                         )
        
        self.obstacle_publisher_ = self.create_publisher(msg_type=ObstacleStatus, 
                                                         topic='/obstacle_status',
                                                         qos_profile=10
                                                        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)
        
        self.get_logger().info(
            f'Obstacle Detector is up and running. '
            f'Threshold: {self.obstacle_threshold:.2f} m'
        )
        
        self.closest_front_x = float('inf')
        self.closest_front_y = float('inf')
        
    
    # def get_lidar_transform(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform(target_frame='base_link', source_frame='base_scan', time=Time())
    #         return transform
        
    #     except Exception as e:
    #         self.get_logger().warn(
    #             f'Could not transform base_scan -> base_link: {e}'
    #         )
    #         return None
        
        
    def transform_lidar_point(self, msg: LaserScan, distance: float, angle: float):
        """Transform a LiDAR measurement from base_scan to base_link."""
        
        point = PointStamped()
        
        point.header.frame_id = 'base_scan'
        point.header.stamp = msg.header.stamp
        
        point.point.x = distance * math.cos(angle)
        point.point.y = distance * math.sin(angle)
        point.point.z = 0.0
        
        transformed_point = self.tf_buffer.transform(object_stamped=point, target_frame='base_link')
        
        return transformed_point
        
        
    
    
    def scan_callback(self, msg):
        
        # transform = self.get_lidar_transform()
        
        # if transform is None:
        #     return
        
        # self.get_logger().info(
        #     f'LiDAR transform: '
        #     f'x={transform.transform.translation.x:.3f}, '
        #     f'y={transform.transform.translation.y:.3f}, '
        #     f'z={transform.transform.translation.z:.3f}'
        # )
        
        front_ranges = []
        left_ranges = []
        right_ranges = []
        
        closest_front_distance = float('inf')
        closest_front_x = float('inf')
        closest_front_y = float('inf')

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
            
            if abs(angle) < math.radians(1.0):
                try:
                    transformed_point = self.transform_lidar_point(
                        msg,
                        distance,
                        angle
                    )

                    self.get_logger().info(
                        f'LiDAR point: '
                        f'x={distance * math.cos(angle):.3f}, '
                        f'y={distance * math.sin(angle):.3f} '
                        f'-> base_link: '
                        f'x={transformed_point.point.x:.3f}, ' # type: ignore
                        f'y={transformed_point.point.y:.3f}' # type: ignore
                    )

                except Exception as e:
                    self.get_logger().warn(
                        f'Point transformation failed: {e}'
                    )

            angle_deg = math.degrees(angle)

            # FRONT: -30° to +30°
            if -30.0 <= angle_deg <= 30.0:
                front_ranges.append(distance)
                
                if distance < closest_front_distance:
                    try:
                        transformed_point = self.transform_lidar_point(
                            msg,
                            distance,
                            angle
                        )

                        closest_front_distance = distance
                        closest_front_x = transformed_point.point.x # type: ignore
                        closest_front_y = transformed_point.point.y # type: ignore

                    except Exception as e:
                        self.get_logger().warn(
                            f'Point transformation failed: {e}'
                        )
                
                if math.isfinite(closest_front_distance):
                    self.get_logger().info(
                        f'Closest front obstacle in base_link: '
                        f'x={closest_front_x:.3f} m, '
                        f'y={closest_front_y:.3f} m, '
                        f'distance={closest_front_distance:.3f} m'
                    )

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
        
        obstacle_detected = (front_distance < self.obstacle_threshold)
        
        status_msg = ObstacleStatus()
        
        status_msg.front_distance = float(front_distance)
        status_msg.left_distance = float(left_distance)
        status_msg.right_distance = float(right_distance)
        status_msg.obstacle_detected = obstacle_detected
        
        self.obstacle_publisher_.publish(status_msg)

        self.get_logger().info(
            f'Front: {front_distance:.2f} m | '
            f'Left: {left_distance:.2f} m | '
            f'Right: {right_distance:.2f} m | '
            f'Obstacle Detected: {obstacle_detected}'
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