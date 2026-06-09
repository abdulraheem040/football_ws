#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Wrench, Pose, Twist, Point, Quaternion
import math


class BallController(Node):
    def __init__(self):
        super().__init__('ball_controller')
        
        self.ball_index = -1
        self.ball_position = [0.0, 0.0, 0.0]
        self.ball_velocity = [0.0, 0.0, 0.0]
        
        # Field boundaries
        self.field_length = 20.0  # from -10 to 10
        self.field_width = 10.0   # from -5 to 5
        self.goal_width = 1.5     # goal opening width
        
        # Respawn cooldown to prevent rapid respawning
        self.respawn_cooldown = 0
        self.respawn_cooldown_max = 50  # 5 seconds at 10Hz
        
        # Gazebo service client for resetting ball position
        self.set_model_state_client = self.create_client(
            SetModelState,
            '/gazebo/set_model_state'
        )
        
        # Subscribe to model states to track ball
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )
        
        # Main control timer
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Ball controller initialized')

    def model_states_callback(self, msg):
        # Find ball in model states
        try:
            ball_index = msg.name.index('football_ball')
            self.ball_index = ball_index
            self.ball_position = [
                msg.pose[ball_index].position.x,
                msg.pose[ball_index].position.y,
                msg.pose[ball_index].position.z
            ]
            
            # Check if ball is out of bounds
            self.check_ball_out_of_bounds()
        except ValueError:
            self.get_logger().warn('Ball not found in model states')

    def check_ball_out_of_bounds(self):
        # Check if ball is out of field boundaries
        # Field: x from -10 to 10, y from -5 to 5
        
        x = self.ball_position[0]
        y = self.ball_position[1]
        
        # Check if ball is outside field (including goal areas)
        out_of_bounds = False
        reason = ""
        
        # Check goal areas first
        if x < -9.5 and abs(y) < self.goal_width:
            out_of_bounds = True
            reason = "GOAL! Red team scores!"
        elif x > 9.5 and abs(y) < self.goal_width:
            out_of_bounds = True
            reason = "GOAL! Blue team scores!"
        # Check if ball went out of bounds on sides
        elif abs(x) > 10.0 or abs(y) > 5.0:
            out_of_bounds = True
            reason = "Ball out of bounds!"
        
        if out_of_bounds and self.respawn_cooldown == 0:
            self.get_logger().info(f'{reason} Respawning ball to center.')
            self.reset_ball()
            self.respawn_cooldown = self.respawn_cooldown_max

    def reset_ball(self):
        # Reset ball to center using Gazebo service
        self.get_logger().info('Resetting ball to center')
        
        # Wait for service to be available
        if not self.set_model_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetModelState service not available')
            return
        
        # Create request
        request = SetModelState.Request()
        request.model_state.model_name = 'football_ball'
        
        # Set position to center
        request.model_state.pose.position.x = 0.0
        request.model_state.pose.position.y = 0.0
        request.model_state.pose.position.z = 0.2
        request.model_state.pose.orientation.w = 1.0
        
        # Reset velocity to zero
        request.model_state.twist.linear.x = 0.0
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0
        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = 0.0
        
        # Call service
        future = self.set_model_state_client.call_async(request)
        
    def control_loop(self):
        # Handle cooldown
        if self.respawn_cooldown > 0:
            self.respawn_cooldown -= 1


def main(args=None):
    rclpy.init(args=args)
    ball_controller = BallController()
    
    try:
        rclpy.spin(ball_controller)
    except KeyboardInterrupt:
        pass
    finally:
        ball_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
