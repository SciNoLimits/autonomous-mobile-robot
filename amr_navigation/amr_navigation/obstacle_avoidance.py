#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
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
        
        # Closest obstacle position in base_link
        self.closest_obstacle_x = float('inf')
        self.closest_obstacle_y = 0.0

        # -------------------------------------------------
        # Publisher
        # -------------------------------------------------

        self.cmd_vel_publisher = self.create_publisher(
            msg_type=TwistStamped,
            topic='/avoidance_cmd',
            qos_profile=10
        )

        # -------------------------------------------------
        # Subscriber
        # -------------------------------------------------

        self.obstacle_subscriber = self.create_subscription(
            msg_type=ObstacleStatus,
            topic='/obstacle_status',
            callback=self.obstacle_callback,
            qos_profile=10
        )

        # -------------------------------------------------
        # Control timer
        # -------------------------------------------------

        self.control_timer = self.create_timer(
            timer_period_sec= 0.1,
            callback=self.control_loop
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
        
        self.closest_obstacle_x = msg.closest_obstacle_x
        self.closest_obstacle_y = msg.closest_obstacle_y
        
        self.get_logger().info(
            f'Closest obstacle in base_link: '
            f'x={self.closest_obstacle_x:.2f} m, '
            f'y={self.closest_obstacle_y:.2f} m'
        )

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
        # if self.left_distance > self.right_distance:

        #     self.state = 'TURN_LEFT'

        #     self.get_logger().info(
        #         'Choosing LEFT for obstacle avoidance.'
        #     )

        # else:

        #     self.state = 'TURN_RIGHT'

        #     self.get_logger().info(
        #         'Choosing RIGHT for obstacle avoidance.'
        #     )
        
        # Choose avoidance direction based on obstacle position
        if self.closest_obstacle_y > 0.0:

            # Obstacle is on the left
            self.state = 'TURN_RIGHT'

            self.get_logger().info(
                f'Obstacle is on the LEFT '
                f'(y={self.closest_obstacle_y:.2f} m). '
                f'Choosing RIGHT.'
            )

        elif self.closest_obstacle_y < 0.0:

            # Obstacle is on the right
            self.state = 'TURN_LEFT'

            self.get_logger().info(
                f'Obstacle is on the RIGHT '
                f'(y={self.closest_obstacle_y:.2f} m). '
                f'Choosing LEFT.'
            )

        else:

            # Obstacle is approximately centered.
            # Fall back to clearance comparison.
            if self.left_distance > self.right_distance:

                self.state = 'TURN_LEFT'

                self.get_logger().info(
                    'Obstacle is centered. '
                    'Choosing LEFT based on clearance.'
                )

            else:

                self.state = 'TURN_RIGHT'

                self.get_logger().info(
                    'Obstacle is centered. '
                    'Choosing RIGHT based on clearance.'
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

        cmd = TwistStamped()
        
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = self.turn_speed

        self.cmd_vel_publisher.publish(cmd)

    def publish_turn_right(self):

        cmd = TwistStamped()
        
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = -self.turn_speed

        self.cmd_vel_publisher.publish(cmd)

    def publish_stop(self):

        cmd = TwistStamped()
        
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

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