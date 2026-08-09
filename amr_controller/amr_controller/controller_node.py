#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from rclpy.executors import ExternalShutdownException

CONTROL_LOOP_FREQUENCY = 10 # Hz

class AMRController(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        # Goal Pose Paramater Declaration
        self.x_goal = self.get_parameter("x_goal").get_parameter_value().double_value
        self.y_goal = self.get_parameter("y_goal").get_parameter_value().double_value
        self.theta_goal = self.get_parameter("theta_goal").get_parameter_value().double_value

        # Controller Gains
        self.k_rho = self.get_parameter("k_rho").get_parameter_value().double_value
        self.k_alpha = self.get_parameter("k_alpha").get_parameter_value().double_value
        self.k_beta = self.get_parameter("k_beta").get_parameter_value().double_value

        # Goal Tolerance
        self.rho_tol = self.get_parameter("rho_tol").get_parameter_value().double_value
        self.rho_tol = self.get_parameter("beta_tol").get_parameter_value().double_value

        # Hardware Limits of TurtleBot3
        self.v_max = self.get_parameter("v_max").get_parameter_value().double_value
        self.w_max = self.get_parameter("w_max").get_parameter_value().double_value
        
        # Robot State - updated by the subscriber  callback
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.yaw = 0.0  # radians
        
        self.subscriber_ = self.create_subscription(msg_type=Odometry, topic='/odom', callback=self.odom_callback, qos_profile=10)
        
        self.publisher_ = self.create_publisher(msg_type=TwistStamped, topic='/cmd_vel', qos_profile=10)
        
        self.get_logger().info(
            message=f"Controller Started. Goal: Pose = ({self.x_goal:.3f}, {self.y_goal:.3f}), Orientation = {self.theta_goal:.3f}"
        )
        
        self.control_timer = self.create_timer(1/CONTROL_LOOP_FREQUENCY,
                                               self.control_loop
                                               )
        
    def odom_callback(self, msg: Odometry):
        pass
    
    
    def control_loop(self):
        pass
    
    
def main(args=None):
    rclpy.init(args=args)
    node = AMRController(name="amr_controller")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()