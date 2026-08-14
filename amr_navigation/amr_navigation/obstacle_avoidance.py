#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from amr_interfaces.msg import ObstacleStatus  # type: ignore
from rclpy.executors import ExternalShutdownException


class ObstacleAvoidance(Node):

    def __init__(self, name: str):
        super().__init__(node_name=name)

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.declare_parameter('obstacle_threshold', 0.50)
        self.declare_parameter('clearance_threshold', 0.70)
        self.declare_parameter('avoidance_speed', 0.15)
        self.declare_parameter('turn_speed', 0.8)

        self.obstacle_threshold = self.get_parameter(
            'obstacle_threshold'
        ).get_parameter_value().double_value

        self.clearance_threshold = self.get_parameter(
            'clearance_threshold'
        ).get_parameter_value().double_value

        self.avoidance_speed = self.get_parameter(
            'avoidance_speed'
        ).get_parameter_value().double_value

        self.turn_speed = self.get_parameter(
            'turn_speed'
        ).get_parameter_value().double_value

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        self.state = 'CLEAR'

        # Latest obstacle measurements
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/avoidance_cmd',
            10
        )

        # -------------------------------------------------
        # Subscriber
        # -------------------------------------------------

        self.obstacle_subscriber = self.create_subscription(
            ObstacleStatus,
            '/obstacle_status',
            self.obstacle_callback,
            10
        )

        # -------------------------------------------------
        # Control timer
        # -------------------------------------------------

        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Obstacle Avoidance node is up and running.'
        )

    # -----------------------------------------------------
    # Obstacle callback
    # -----------------------------------------------------

    def obstacle_callback(self, msg: ObstacleStatus):

        self.front_distance = msg.front_distance
        self.left_distance = msg.left_distance
        self.right_distance = msg.right_distance

    # -----------------------------------------------------
    # State machine
    # -----------------------------------------------------

    def control_loop(self):

        if self.state == 'CLEAR':

            self.handle_clear_state()

        elif self.state == 'TURN_LEFT':

            self.handle_turn_left_state()

        elif self.state == 'TURN_RIGHT':

            self.handle_turn_right_state()

        elif self.state == 'STOP':

            self.handle_stop_state()

    # -----------------------------------------------------
    # CLEAR
    # -----------------------------------------------------

    def handle_clear_state(self):

        # No obstacle in front
        if self.front_distance > self.obstacle_threshold:

            self.publish_stop()

            return

        # Obstacle detected
        self.get_logger().warn(
            f'Obstacle detected: '
            f'front={self.front_distance:.2f} m'
        )

        # Choose the side with more clearance
        if self.left_distance > self.right_distance:

            self.state = 'TURN_LEFT'

            self.get_logger().info(
                'Choosing LEFT for obstacle avoidance.'
            )

        else:

            self.state = 'TURN_RIGHT'

            self.get_logger().info(
                'Choosing RIGHT for obstacle avoidance.'
            )

    # -----------------------------------------------------
    # TURN LEFT
    # -----------------------------------------------------

    def handle_turn_left_state(self):

        # Continue turning until front is clear
        if self.front_distance < self.clearance_threshold:

            self.publish_turn_left()

        else:

            self.state = 'CLEAR'

            self.publish_stop()

            self.get_logger().info(
                'Front clear. Returning to CLEAR state.'
            )

    # -----------------------------------------------------
    # TURN RIGHT
    # -----------------------------------------------------

    def handle_turn_right_state(self):

        # Continue turning until front is clear
        if self.front_distance < self.clearance_threshold:

            self.publish_turn_right()

        else:

            self.state = 'CLEAR'

            self.publish_stop()

            self.get_logger().info(
                'Front clear. Returning to CLEAR state.'
            )

    # -----------------------------------------------------
    # STOP
    # -----------------------------------------------------

    def handle_stop_state(self):

        self.publish_stop()

    # -----------------------------------------------------
    # Velocity commands
    # -----------------------------------------------------

    def publish_turn_left(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = self.turn_speed

        self.cmd_vel_publisher.publish(cmd)

    def publish_turn_right(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = -self.turn_speed

        self.cmd_vel_publisher.publish(cmd)

    def publish_stop(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0

        self.cmd_vel_publisher.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance(name="obstacle_avoidance")

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()