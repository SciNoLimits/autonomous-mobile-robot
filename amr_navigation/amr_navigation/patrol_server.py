#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from amr_interfaces.action import NavigateWaypoints, Patrol # type: ignore



class PatrolServer(Node):
    def __init__(self, name: str):
        super().__init__(node_name=name)
        
        self.callback_group = ReentrantCallbackGroup()
        
        self.navigate_client = ActionClient(node=self, 
                                            action_name='navigate_waypoints', 
                                            action_type=NavigateWaypoints, 
                                            callback_group=self.callback_group
                                            )
        
        self.patrol_server = ActionServer(node=self,
                                          action_type=Patrol,
                                          action_name='patrol',
                                          execute_callback=self.execute_callback,
                                          goal_callback=self.goal_callback,
                                          cancel_callback=self.cancel_callback,
                                          callback_group=ReentrantCallbackGroup()
                                        )
        
        self.get_logger().info("Patrol Server is up and running.")
        
        
    # ---------------------------------------------------------
    # Goal handling
    # ---------------------------------------------------------

    def goal_callback(self, goal_request):
        """Accept valid patrol goals."""

        if len(goal_request.waypoints) == 0:
            self.get_logger().warn(
                'Rejected patrol goal: no waypoints provided.'
            )
            return GoalResponse.REJECT

        if goal_request.patrol_cycles == 0:
            self.get_logger().warn(
                'Rejected patrol goal: patrol_cycles must be greater than 0.'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            f'Accepted patrol request: '
            f'{len(goal_request.waypoints)} waypoints, '
            f'{goal_request.patrol_cycles} cycles.'
        )

        return GoalResponse.ACCEPT
    
    
    # ---------------------------------------------------------
    # Cancellation
    # ---------------------------------------------------------

    def cancel_callback(self, goal_handle):
        """Allow patrol goals to be cancelled."""

        self.get_logger().info(
            'Patrol cancellation requested.'
        )

        return CancelResponse.ACCEPT
    
    
    # ---------------------------------------------------------
    # Navigation feedback
    # ---------------------------------------------------------

    def navigation_feedback_callback(
        self,
        feedback_msg,
        patrol_goal_handle,
        cycle_number
    ):
        """Forward waypoint progress as patrol feedback."""

        feedback = Patrol.Feedback()

        feedback.current_cycle = cycle_number
        feedback.current_waypoint = (
            feedback_msg.feedback.current_waypoint
        )

        patrol_goal_handle.publish_feedback(feedback)
        
        
    # ---------------------------------------------------------
    # Patrol execution
    # ---------------------------------------------------------

    def execute_callback(self, goal_handle):

        self.get_logger().info(
            'Executing patrol mission.'
        )

        waypoints = goal_handle.request.waypoints
        patrol_cycles = goal_handle.request.patrol_cycles

        result = Patrol.Result()

        cycles_completed = 0

        # Wait for the navigation server
        if not self.navigate_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                'NavigateWaypoints Action Server is not available.'
            )

            goal_handle.abort()

            result.success = False
            result.cycles_completed = 0
            result.message = (
                'NavigateWaypoints Action Server unavailable.'
            )

            return result

        # -----------------------------------------------------
        # Execute patrol cycles
        # -----------------------------------------------------

        for cycle in range(1, patrol_cycles + 1):

            self.get_logger().info(
                f'Starting patrol cycle '
                f'{cycle}/{patrol_cycles}.'
            )

            # Check cancellation before starting a cycle
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Patrol cancelled before starting navigation.')

                goal_handle.canceled()

                result.success = False
                result.cycles_completed = cycles_completed
                result.message = 'Patrol cancelled.'

                return result

            # ---------------------------------------------
            # Create navigation goal
            # ---------------------------------------------

            navigation_goal = NavigateWaypoints.Goal()

            navigation_goal.waypoints = waypoints

            # ---------------------------------------------
            # Send navigation goal
            # ---------------------------------------------

            send_goal_future = (
                self.navigate_client.send_goal_async(
                    navigation_goal,
                    feedback_callback=lambda feedback:
                    self.navigation_feedback_callback(
                        feedback,
                        goal_handle,
                        cycle
                    )
                )
            )

            # Wait for goal response
            while rclpy.ok() and not send_goal_future.done():

                if goal_handle.is_cancel_requested:
                    self.get_logger().info(
                        'Patrol cancelled while sending navigation goal.'
                    )

                    goal_handle.canceled()

                    result.success = False
                    result.cycles_completed = cycles_completed
                    result.message = 'Patrol cancelled.'

                    return result

                time.sleep(0.05)

            navigation_goal_handle = send_goal_future.result()

            if navigation_goal_handle is None:
                self.get_logger().error(
                    'Navigation goal was rejected.'
                )

                goal_handle.abort()

                result.success = False
                result.cycles_completed = cycles_completed
                result.message = (
                    'Navigation goal rejected.'
                )

                return result

            # ---------------------------------------------
            # Wait for navigation result
            # ---------------------------------------------

            result_future = (
                navigation_goal_handle.get_result_async()
            )

            # while rclpy.ok():
                
            #     if goal_handle.is_cancel_requested:

            #         self.get_logger().info(
            #             'Cancelling active navigation goal.'
            #         )

            #         navigation_goal_handle.cancel_goal_async()

            #         goal_handle.canceled()

            #         result.success = False
            #         result.cycles_completed = cycles_completed
            #         result.message = 'Patrol cancelled.'

            #         return result

            #     # if goal_handle.is_cancel_requested:

            #     #     self.get_logger().info(
            #     #         'Cancelling active navigation goal.'
            #     #     )

            #     #     cancel_future = (
            #     #         navigation_goal_handle.cancel_goal_async()
            #     #     )

            #     #     rclpy.spin_until_future_complete(
            #     #         self,
            #     #         cancel_future
            #     #     )

            #     #     goal_handle.canceled()

            #     #     result.success = False
            #     #     result.cycles_completed = cycles_completed
            #     #     result.message = 'Patrol cancelled.'

            #     #     return result

            #     if result_future.done():
            #         break

            #     # Allow callbacks and feedback to execute
            #     rclpy.spin_once(
            #         self,
            #         timeout_sec=0.1
            #     )
            
            while rclpy.ok() and not result_future.done():

                if goal_handle.is_cancel_requested:

                    self.get_logger().info(
                        'Cancelling active navigation goal.'
                    )

                    cancel_future = (
                        navigation_goal_handle.cancel_goal_async()
                    )

                    # Wait for the navigation server to acknowledge
                    # the cancellation request.
                    while rclpy.ok() and not cancel_future.done():
                        time.sleep(0.05)

                    self.get_logger().info(
                        'Navigation cancellation request processed.'
                    )

                    goal_handle.canceled()

                    result.success = False
                    result.cycles_completed = cycles_completed
                    result.message = 'Patrol cancelled.'

                    return result

                time.sleep(0.05)

            navigation_result = result_future.result().result

            # ---------------------------------------------
            # Check navigation result
            # ---------------------------------------------

            if not navigation_result.success:

                self.get_logger().error(
                    f'Navigation failed during cycle {cycle}.'
                )

                goal_handle.abort()

                result.success = False
                result.cycles_completed = cycles_completed
                result.message = (
                    f'Navigation failed during cycle {cycle}.'
                )

                return result

            cycles_completed += 1

            self.get_logger().info(
                f'Patrol cycle {cycle} completed.'
            )

        # -----------------------------------------------------
        # Patrol completed
        # -----------------------------------------------------

        goal_handle.succeed()

        result.success = True
        result.cycles_completed = cycles_completed
        result.message = (
            'Patrol completed successfully.'
        )

        self.get_logger().info(
            'Patrol mission completed.'
        )

        return result
    
    
def main(args=None):

    rclpy.init(args=args)

    node = PatrolServer(
        name='patrol_server'
    )

    executor = MultiThreadedExecutor()

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()          