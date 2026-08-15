#!/usr/bin/env python3

from enum import Enum

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from amr_interfaces.msg import ObstacleStatus  # type: ignore
from rclpy.executors import ExternalShutdownException


ARBITRATION_LOOP_FREQUENCY = 5  # Hz


class NavigationState(Enum):
    FOLLOW_GOAL = 1
    AVOID_OBSTACLE = 2
    REJOIN_NAVIGATION = 3
    EMERGENCY_STOP = 4
    

class CmdArbitrator(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.state = NavigationState.FOLLOW_GOAL
        self.previous_state = self.state
        
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
        
        
    def set_state(self, new_state: NavigationState):
        """Set the current state and log the transition if it changes."""
        
        if new_state != self.state:
            self.get_logger().info(f'State transition: {self.state.name} -> {new_state.name}')
            self.previous_state = self.state
            self.state = new_state
        
        
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
        """Select the command according to the navigation state."""

        # --------------------------------------------------
        # State transitions
        # --------------------------------------------------

        # CRITICAL SAFETY CHECK: If an object is too close, instantly trigger emergency stop.
        # This overrides all other states.
        if self.front_distance < 0.20:

            self.set_state(
                NavigationState.EMERGENCY_STOP
            )

        # RECOVERY FROM EMERGENCY: If currently stopped, check if the path is clear to resume.
        elif self.state == NavigationState.EMERGENCY_STOP:

            # Only attempt recovery if the immediate hazard has moved away (> 20cm)
            if self.front_distance >= 0.20:

                # If a minor obstacle is still detected further out, switch to avoidance mode
                if self.obstacle_detected:
                    self.set_state(
                        NavigationState.AVOID_OBSTACLE
                    )
                # If the path is entirely clear, go back to normal goal tracking
                else:
                    self.set_state(
                        NavigationState.FOLLOW_GOAL
                    )

        # NORMAL MODE: Moving toward the goal, but watching out for obstacles.
        elif self.state == NavigationState.FOLLOW_GOAL:

            # If a path block is detected, immediately switch to the avoidance algorithm
            if self.obstacle_detected:

                self.set_state(
                    NavigationState.AVOID_OBSTACLE
                )

        # AVOIDANCE MODE: Executing steering maneuvers around an object.
        elif self.state == NavigationState.AVOID_OBSTACLE:

            # Once the obstacle is cleared from sensors, begin returning to the main path
            if not self.obstacle_detected:

                self.set_state(
                    NavigationState.REJOIN_NAVIGATION
                )

        # RECOVERY PATHING: Aligning the robot back with the original planned route.
        elif self.state == NavigationState.REJOIN_NAVIGATION:

            # If a new obstacle appears while trying to rejoin, go back to avoiding
            if self.obstacle_detected:

                self.set_state(
                    NavigationState.AVOID_OBSTACLE
                )

            # If the rejoining path remains clear, successfully resume normal goal tracking
            else:

                self.set_state(
                    NavigationState.FOLLOW_GOAL
                )

        # --------------------------------------------------
        # Command selection
        # --------------------------------------------------

        # Initialize a blank ROS 2 velocity message template
        cmd = TwistStamped()

        # Apply standard navigation speeds if driving toward the target
        if self.state == NavigationState.FOLLOW_GOAL:

            cmd = self.navigation_cmd

        # Apply specialized steering speeds if dodging an obstacle
        elif self.state == NavigationState.AVOID_OBSTACLE:

            cmd = self.avoidance_cmd

        # Apply standard navigation speeds to maneuver back onto the main path
        elif self.state == NavigationState.REJOIN_NAVIGATION:

            cmd = self.navigation_cmd

        # Manually zero out all forces to freeze the robot in place safely
        elif self.state == NavigationState.EMERGENCY_STOP:

            # Add current ROS time and frame context to the message header
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = 'base_link'

            # Force all straight-line movements (X, Y, Z) to zero
            cmd.twist.linear.x = 0.0
            cmd.twist.linear.y = 0.0
            cmd.twist.linear.z = 0.0

            # Force all rotational movements (roll, pitch, yaw) to zero
            cmd.twist.angular.x = 0.0
            cmd.twist.angular.y = 0.0
            cmd.twist.angular.z = 0.0

        # Actively send the final determined movement velocity to the robot's hardware motors
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