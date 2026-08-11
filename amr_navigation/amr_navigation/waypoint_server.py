#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from amr_interfaces.action import NavigateWaypoints # type: ignore

class WaypointServer(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.current_x = 0.0
        self.current_y = 0.0
        
        self.goal_publisher_ = self.create_publisher(msg_type=Pose2D, topic='/amr_controller/goal', qos_profile=10)
        
        self.odom_subscriber_ = self.create_subscription(msg_type=Odometry, topic='/odom', callback=self.odom_callback, qos_profile=10)
        
        self.goal_reached_subscriber_ = self.create_subscription(msg_type=Bool, topic='/amr_controller/goal_reached', callback=self.goal_reached_callback, qos_profile=10)
        
        self.action_server = ActionServer(node=self,
                                          action_type=NavigateWaypoints,
                                          action_name='navigate_waypoints',
                                          execute_callback=self.execute_callback,
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          callback_group=ReentrantCallbackGroup()
                                          )
        
        self.get_logger().info("Waypoint Server is up and running.")
       
       
    def goal_reached_callback(self, msg: Bool):
        """Receive goal completion status from the controller."""
        self.goal_reached = msg.data
        
        
    def odom_callback(self, msg: Odometry):
        """Update the robot's current position."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
    def goal_callback(self, goal_request):
        """Accept a navigation goal if it contains waypoints."""

        if len(goal_request.waypoints) == 0:
            self.get_logger().warn(
                'Rejected goal: no waypoints provided.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted navigation goal with '
            f'{len(goal_request.waypoints)} waypoints.'
        )

        return GoalResponse.ACCEPT
    
    
    def cancel_callback(self, goal_handle):
        """Allow navigation goals to be cancelled."""

        self.get_logger().info('Navigation cancellation requested.')

        return CancelResponse.ACCEPT
    
    
    def distance_to_waypoint(self, waypoint):
        """Calculate planar distance to a waypoint."""

        dx = waypoint.x - self.current_x
        dy = waypoint.y - self.current_y

        return math.hypot(dx, dy)
    
    
    def publish_goal(self, waypoint):
        """Send a waypoint to the low-level controller."""

        goal = Pose2D()

        goal.x = waypoint.x
        goal.y = waypoint.y
        goal.theta = waypoint.theta

        self.goal_publisher_.publish(goal)

        self.get_logger().info(
            f'Sending waypoint: '
            f'({goal.x:.2f}, {goal.y:.2f}, {goal.theta:.2f})'
        )
        
        
    def stop_robot(self):
        """Tell the controller to hold the current position."""

        goal = Pose2D()

        goal.x = self.current_x
        goal.y = self.current_y
        goal.theta = 0.0

        self.goal_publisher_.publish(goal)
    
        
    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing navigation goal.')

        waypoints = goal_handle.request.waypoints

        feedback_msg = NavigateWaypoints.Feedback()
        result = NavigateWaypoints.Result()

        completed = 0

        for index, waypoint in enumerate(waypoints):

            if goal_handle.is_cancel_requested:
                self.stop_robot()
                goal_handle.canceled()

                result.success = False
                result.waypoints_completed = completed
                result.message = 'Navigation cancelled.'

                return result
            
            self.goal_reached = False

            self.publish_goal(waypoint)

            while rclpy.ok():

                if goal_handle.is_cancel_requested:
                    self.stop_robot()
                    goal_handle.canceled()

                    result.success = False
                    result.waypoints_completed = completed
                    result.message = 'Navigation cancelled.'

                    return result

                distance = self.distance_to_waypoint(waypoint)

                feedback_msg.current_waypoint = index + 1
                feedback_msg.distance_remaining = float(distance)

                goal_handle.publish_feedback(feedback_msg)

                if self.goal_reached:
                    break

                time.sleep(0.1)

            completed += 1

            self.get_logger().info(
                f'Waypoint {index + 1} reached.'
            )

        self.stop_robot()

        goal_handle.succeed()

        result.success = True
        result.waypoints_completed = completed
        result.message = 'All waypoints reached successfully.'

        self.get_logger().info(
            'Navigation mission completed.'
        )

        return result
    
    
def main(args=None):

    rclpy.init(args=args)

    node = WaypointServer(name="waypoint_server")

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()