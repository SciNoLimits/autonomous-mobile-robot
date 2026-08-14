#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from amr_interfaces.msg import ObstacleStatus  # type: ignore
from rclpy.executors import ExternalShutdownException


ARBITRATION_LOOP_FREQUENCY = 5  # Hz


class CmdArbitrator(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.navigation_cmd = TwistStamped()
        self.avoidance_cmd = TwistStamped()
        
        self.obstacle_detected = False
        self.front_distance = float('inf')
        
        self.navigation_subscriber_ = self.create_subscription(
            msg_type=TwistStamped,
            topic='/navigation_cmd',
            callback=self.navigation_callback,
            qos_profile=10
        )
        
        self.avoidance_subscriber_ = self.create_subscription(
            msg_type=TwistStamped,
            topic='/avoidance_cmd',
            callback=self.avoidance_callback,
            qos_profile=10
        )
        
        self.obstacle_subscriber_ = self.create_subscription(
            msg_type=ObstacleStatus,
            topic='/obstacle_status',
            callback=self.obstacle_callback,
            qos_profile=10
        )
        
        self.cmd_vel_publisher_ = self.create_publisher(
            msg_type=TwistStamped,
            topic='/cmd_vel',
            qos_profile=10
        )
        
        self.control_timer_ = self.create_timer(1/ARBITRATION_LOOP_FREQUENCY, self.arbitration_callback)
        
        self.get_logger().info('Command Arbitrator is up and running.')
        
        
    def navigation_callback(self, msg: TwistStamped):
        """Store the latest navigation command."""
        self.navigation_cmd = msg
        
        
    def avoidance_callback(self, msg: TwistStamped):
        """Store the latest obstacle avoidance command."""
        self.avoidance_cmd = msg
        
        
    def obstacle_callback(self, msg: ObstacleStatus):
        """Update the obstacle detection status based on the received message."""
        self.obstacle_detected = msg.obstacle_detected
        self.front_distance = msg.front_distance
        
        
    def arbitration_callback(self):
        """Select the command that should control the robot."""
        
        cmd = TwistStamped()
        
        # Emergency stop if an obstacle is detected within 0.20 meters
        if self.front_distance < 0.20:
            self.get_logger().warn(
                f'Emergency stop: front obstacle at '
                f'{self.front_distance:.2f} m'
            )
            
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'
            
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            
        # If an obstacle is detected, use the avoidance command
        elif self.obstacle_detected:
            cmd = self.avoidance_cmd
            
        # If no obstacle is detected, use the navigation command
        else:
            cmd = self.navigation_cmd
            
        # Publish the selected command
        self.cmd_vel_publisher_.publish(cmd)
        

def main(args=None):
    rclpy.init(args=args)
    node = CmdArbitrator(name='cmd_arbitrator')
    
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