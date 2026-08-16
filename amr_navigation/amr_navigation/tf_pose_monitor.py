#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
from rclpy.executors import ExternalShutdownException


TIMER_FREQUENCY = 1  # Hz


class TfPoseMonitor(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(buffer=self.tf_buffer, node=self)
        
        self.timer = self.create_timer(
            timer_period_sec=1.0 / TIMER_FREQUENCY,
            callback=self.timer_callback
        )
        
        self.get_logger().info("TF Pose Monitor is up and running.")
        
    
    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(target_frame='odom', source_frame='base_link', time=Time())
            
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            
            q = transform.transform.rotation
            
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            
            self.get_logger().info(
                f'Robot pose from TF: '
                f'x={x:.3f}, '
                f'y={y:.3f}, '
                f'yaw={yaw:.3f} rad'
            )
            
        except Exception as e:
            self.get_logger().warn(
                f'Could not get odom -> base_link transform: {e}'
            )
            
            
def main(args=None):
    rclpy.init(args=args)
    node = TfPoseMonitor(name="tf_pose_monitor")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        

if __name__ == "__main__":
    main()