#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
from rclpy.executors import ExternalShutdownException

CONTROL_LOOP_FREQUENCY = 10 # Hz

class AMRController(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        # Register the cleanup callback before spin handles any shutdown events
        rclpy.get_default_context().on_shutdown(self.stop_robot)
        
        # Goal Pose Paramater Declaration
        self.declare_parameter("x_goal", 1.0)       # meters
        self.declare_parameter("y_goal", 1.0)       # meters
        self.declare_parameter("theta_goal", 0.0)   # radians
        
        self.x_goal = self.get_parameter("x_goal").get_parameter_value().double_value
        self.y_goal = self.get_parameter("y_goal").get_parameter_value().double_value
        self.theta_goal = self.get_parameter("theta_goal").get_parameter_value().double_value

        # Controller Gains
        self.declare_parameter("k_rho", 0.4)
        self.declare_parameter("k_alpha", 0.8)
        self.declare_parameter("k_beta", -0.15)
        
        self.k_rho = self.get_parameter("k_rho").get_parameter_value().double_value
        self.k_alpha = self.get_parameter("k_alpha").get_parameter_value().double_value
        self.k_beta = self.get_parameter("k_beta").get_parameter_value().double_value

        # Goal Tolerance
        self.declare_parameter("rho_tol", 0.05)  # Stop within 5 cm
        self.declare_parameter("beta_tol", 0.02)  # Stop within 0.02 radians (~1.15 degrees)
        
        self.rho_tol = self.get_parameter("rho_tol").get_parameter_value().double_value
        self.beta_tol = self.get_parameter("beta_tol").get_parameter_value().double_value

        # Hardware Limits of TurtleBot3
        self.declare_parameter("v_max", 0.22)  # m/s
        self.declare_parameter("w_max", 2.84)  # rad/s
        
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
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.yaw = yaw
        
    
    def compute_error(self):
        """
        Compute polar error variables (rho , alpha , beta ).
        """
        dx = self.x_goal - self.x
        dy = self.y_goal - self.y

        rho = math.hypot(dx, dy)
        phi = math.atan2(dy, dx)
        alpha = phi - self.yaw
        beta = self.theta_goal - phi

        # Normalise to ( -pi , pi]
        alpha = math.atan2(math.sin(alpha), math.cos(alpha))
        beta = math.atan2(math.sin(beta), math.cos(beta))

        return rho, alpha, beta
    
    
    def compute_control(self, rho, alpha, beta):
        """
        Compute control commands (v , w) based on polar error variables (rho , alpha , beta ).
        """
        v = self.k_rho * rho
        w = self.k_alpha * alpha + self.k_beta * beta

        # Reverse if goal is behind the robot
        if abs(alpha) > math.pi / 2:
            v = -v

        return v, w
    
    
    def saturate(self, v, w):
        """Proportional scaling to respect hardware limits"""
        scale = min(1.0, self.v_max / (abs(v) + 1e-9), self.w_max / (abs(w) + 1e-9))
        return scale * v, scale * w
    
    
    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(cmd)
        self.get_logger().info("Robot Stopped.")
    
    
    def control_loop(self):
        """Main control loop that computes and publishes control commands based on the current robot state and goal."""
        rho, alpha, beta = self.compute_error()
        
        # if rho < self.rho_tol and abs(beta) < self.beta_tol:
        #     self.stop_robot()
        #     self.control_timer.cancel()
        #     self.get_logger().info(f"Goal reached. rho = {rho:.4f}, beta = {beta:.4f}")
        #     return
        
        if rho < self.rho_tol:
            if abs(beta) < self.beta_tol:
                self.stop_robot()
                self.control_timer.cancel()
                self.get_logger().info(f"Goal reached. rho = {rho:.4f}, beta = {beta:.4f}")
                return
            else:
                # Aligning to the goal orientation
                v = 0.0
                w = self.k_beta * beta
                w = max(-self.w_max, min(self.w_max, w))  # Saturate angular velocity

        
        v, w = self.compute_control(rho, alpha, beta)
        v, w = self.saturate(v, w)
        
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "base_link"
        
        cmd.twist.linear.x = v
        cmd.twist.angular.z = w
        self.publisher_.publish(cmd)
        
        self.get_logger().info(f"rho ={rho:.3f} alpha ={alpha:.3f} beta ={beta:.3f} v ={v:.3f} w ={w:.3f}",
                                throttle_duration_sec=1.0
                                )
    
    
def main(args=None):
    rclpy.init(args=args)
    node = AMRController(name="amr_controller")

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