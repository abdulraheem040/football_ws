#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from rclpy.executors import MultiThreadedExecutor
import math
import random


class RobotController(Node):

    def __init__(self, robot_name='robot'):

        super().__init__(f'{robot_name}_controller')

        self.robot_name = robot_name

        # Robot pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Ball pose
        self.ball_x = 0.0
        self.ball_y = 0.0

        # Ball detection
        self.ball_detected = False
        self.ball_distance = 0.0
        self.ball_angle = 0.0

        # Opponent robot pose
        self.opponent_x = 0.0
        self.opponent_y = 0.0
        self.opponent_detected = False

        # Goal positions - kick to opposite goal
        if robot_name == 'robot_blue':
            self.target_goal_x = -10.0
        else:
            self.target_goal_x = 10.0

        self.target_goal_y = 0.0

        # =========================================================
        # NEW: KICK STATE MACHINE
        # =========================================================
        # States: 'approaching', 'positioning', 'ready_to_kick', 'kicking', 'cooldown', 'carrying'
        self.kick_state = 'approaching'
        self.kick_timer = 0
        self.kick_cooldown = 0
        self.carry_timer = 0
        self.carry_angle = 0.0

        # Random speed parameters for varied gameplay possibilities
        # Linear speed: 0.5 to 2.0 m/s
        self.linear_speed = random.uniform(0.5, 2.0)
        # Angular speed: 0.8 to 2.5 rad/s
        self.angular_speed = random.uniform(0.8, 2.5)
        # Kick speed: 0.6 to 1.8 m/s
        self.kick_speed = random.uniform(0.6, 1.8)
        
        self.get_logger().info(
            f'{robot_name} speeds - Linear: {self.linear_speed:.2f}, '
            f'Angular: {self.angular_speed:.2f}, Kick: {self.kick_speed:.2f}'
        )

        # Positioning tolerances (must center before kick)
        self.angle_tolerance = 0.1  # radians (~5.7 degrees)
        self.distance_tolerance = 0.3  # meters

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            f'/{robot_name}/cmd_vel',
            10
        )

        # Subscribers
        self.create_subscription(
            Odometry,
            f'/{robot_name}/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            ModelStates,
            '/model_states',
            self.model_states_callback,
            10
        )

        # Main control timer
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f'{robot_name} controller started')

    # =========================================================
    # ODOM CALLBACK
    # =========================================================
    def odom_callback(self, msg):

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # =========================================================
    # BALL POSITION CALLBACK
    # =========================================================
    def model_states_callback(self, msg):

        try:
            index = msg.name.index('football_ball')

            self.ball_x = msg.pose[index].position.x
            self.ball_y = msg.pose[index].position.y

        except ValueError:
            pass

        # Track opponent robot position
        opponent_name = 'robot_red' if self.robot_name == 'robot_blue' else 'robot_blue'
        try:
            index = msg.name.index(opponent_name)
            self.opponent_x = msg.pose[index].position.x
            self.opponent_y = msg.pose[index].position.y
            self.opponent_detected = True
        except ValueError:
            self.opponent_detected = False

    # =========================================================
    # DETECT BALL
    # =========================================================
    def detect_ball(self):

        dx = self.ball_x - self.current_x
        dy = self.ball_y - self.current_y

        self.ball_distance = math.sqrt(dx * dx + dy * dy)

        angle = math.atan2(dy, dx)

        self.ball_angle = angle - self.current_yaw

        # Normalize angle
        self.ball_angle = math.atan2(
            math.sin(self.ball_angle),
            math.cos(self.ball_angle)
        )

        self.ball_detected = self.ball_distance < 15.0

    # =========================================================
    # DETECT OPPONENT BLOCKING GOAL
    # =========================================================
    def is_opponent_blocking_goal(self):
        if not self.opponent_detected:
            return False
        
        # Calculate angle to goal
        dx_goal = self.target_goal_x - self.current_x
        dy_goal = self.target_goal_y - self.current_y
        angle_to_goal = math.atan2(dy_goal, dx_goal)
        
        # Calculate angle to opponent
        dx_opp = self.opponent_x - self.current_x
        dy_opp = self.opponent_y - self.current_y
        angle_to_opponent = math.atan2(dy_opp, dx_opp)
        
        # Calculate distance to opponent
        dist_to_opponent = math.sqrt(dx_opp * dx_opp + dy_opp * dy_opp)
        
        # Check if opponent is in front and blocking path to goal
        # Opponent is blocking if:
        # 1. Within 5 meters
        # 2. Angle to opponent is similar to angle to goal (within 30 degrees)
        # 3. Opponent is closer to goal than we are
        
        angle_diff = abs(angle_to_opponent - angle_to_goal)
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        dist_to_goal = math.sqrt(dx_goal * dx_goal + dy_goal * dy_goal)
        opp_dist_to_goal = math.sqrt(
            (self.target_goal_x - self.opponent_x) ** 2 + 
            (self.target_goal_y - self.opponent_y) ** 2
        )
        
        blocking = (dist_to_opponent < 5.0 and 
                    abs(angle_diff) < 0.5 and  # ~30 degrees
                    opp_dist_to_goal < dist_to_goal)
        
        return blocking

    # =========================================================
    # MOVE TO BALL WITH POSITIONING STATE MACHINE
    # =========================================================
    def move_to_ball(self):

        twist = Twist()

        # =========================================================
        # STATE: COOLDOWN (after kick, stop action)
        # =========================================================
        if self.kick_cooldown > 0:
            self.kick_cooldown -= 1
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.ball_detected = False
            self.cmd_vel_pub.publish(twist)
            
            # Exit cooldown and return to approaching
            if self.kick_cooldown == 0:
                self.kick_state = 'approaching'
            return

        # =========================================================
        # STATE: KICKING (forward motion with force)
        # =========================================================
        if self.kick_state == 'kicking':
            self.kick_timer += 1

            if self.kick_timer > 5:  # Kick for 5 ticks
                # Kick complete, enter cooldown
                self.kick_timer = 0
                self.kick_cooldown = 100
                self.ball_detected = False
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                # Strong forward kick
                twist.linear.x = self.kick_speed * 3.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            return

        # =========================================================
        # STATE: READY_TO_KICK (centered and aligned - now kick!)
        # =========================================================
        if self.kick_state == 'ready_to_kick':
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            
            # Transition to kicking
            self.kick_state = 'kicking'
            self.kick_timer = 0
            return

        # =========================================================
        # STATE: POSITIONING (fine-tune angle and distance)
        # =========================================================
        if self.kick_state == 'positioning':
            if not self.ball_detected:
                # Lost ball, go back to approaching
                self.kick_state = 'approaching'
                twist.angular.z = self.angular_speed * 0.5
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Check if properly centered
            angle_ok = abs(self.ball_angle) < self.angle_tolerance
            distance_ok = 0.4 < self.ball_distance < 1.0

            if angle_ok and distance_ok:
                # Check if opponent is blocking the goal
                if self.is_opponent_blocking_goal():
                    self.get_logger().info(
                        f'{self.robot_name}: Opponent blocking goal! Entering CARRY state.'
                    )
                    self.kick_state = 'carrying'
                    self.carry_timer = 0
                    # Choose random carry angle (left or right)
                    self.carry_angle = random.choice([0.5, -0.5])  # ~30 degrees
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    return
                else:
                    # Properly positioned! Ready to kick
                    self.get_logger().info(
                        f'{self.robot_name}: Ball centered. Angle: {self.ball_angle:.3f}, '
                        f'Distance: {self.ball_distance:.3f}. KICKING NOW!'
                    )
                    self.kick_state = 'ready_to_kick'
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist)
                    return

            # Still positioning - fine adjustments
            
            # If angle is off, rotate first
            if abs(self.ball_angle) > self.angle_tolerance:
                angular_speed = self.angular_speed * self.ball_angle * 0.5  # Slower rotation
                angular_speed = max(min(angular_speed, 0.5), -0.5)  # Clamp for precision
                twist.angular.z = angular_speed
                twist.linear.x = 0.05  # Tiny forward motion
                
                self.get_logger().debug(
                    f'{self.robot_name}: Positioning - turning. Angle: {self.ball_angle:.3f}'
                )

            # If distance is off, adjust forward/backward
            elif self.ball_distance < 0.4:
                # Too close, back up slightly
                twist.linear.x = -0.1
                twist.angular.z = 0.0
                self.get_logger().debug(
                    f'{self.robot_name}: Positioning - too close, backing up. Distance: {self.ball_distance:.3f}'
                )

            elif self.ball_distance > 1.0:
                # Too far, move forward
                twist.linear.x = self.linear_speed * 0.3
                twist.angular.z = 0.0
                self.get_logger().debug(
                    f'{self.robot_name}: Positioning - moving closer. Distance: {self.ball_distance:.3f}'
                )

            else:
                # Distance is good, just wait
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            self.cmd_vel_pub.publish(twist)
            return

        # =========================================================
        # STATE: CARRYING (dribble ball at angle to avoid blocker)
        # =========================================================
        if self.kick_state == 'carrying':
            self.carry_timer += 1
            
            # Carry for 30 ticks (3 seconds) then kick
            if self.carry_timer > 30:
                self.get_logger().info(
                    f'{self.robot_name}: Carry complete. Kicking towards goal!'
                )
                self.kick_state = 'ready_to_kick'
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return
            
            # Move at angle while keeping ball close
            # Slow forward motion with angular component
            twist.linear.x = self.linear_speed * 0.4
            twist.angular.z = self.carry_angle * 0.3
            
            self.cmd_vel_pub.publish(twist)
            return

        # =========================================================
        # STATE: APPROACHING (move towards ball)
        # =========================================================
        if self.kick_state == 'approaching':
            if not self.ball_detected:
                # Search for ball
                twist.angular.z = self.angular_speed * 0.5
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            # Turn towards ball first
            if abs(self.ball_angle) > 0.2:
                angular_speed = self.angular_speed * self.ball_angle
                angular_speed = max(min(angular_speed, 1.0), -1.0)
                twist.angular.z = angular_speed
                twist.linear.x = 0.1
                
            # Approaching distance
            elif self.ball_distance > 1.0:
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                
            # Getting close - transition to positioning
            elif self.ball_distance <= 1.0:
                self.get_logger().info(
                    f'{self.robot_name}: Close to ball ({self.ball_distance:.3f}m). '
                    f'Entering POSITIONING state for precise centering.'
                )
                self.kick_state = 'positioning'
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                return

            self.cmd_vel_pub.publish(twist)
            return

    # =========================================================
    # CONTROL LOOP
    # =========================================================
    def control_loop(self):

        self.detect_ball()
        self.move_to_ball()


# =============================================================
# MAIN
# =============================================================
def main(args=None):

    rclpy.init(args=args)

    robot_blue = RobotController('robot_blue')
    robot_red = RobotController('robot_red')

    executor = MultiThreadedExecutor()

    executor.add_node(robot_blue)
    executor.add_node(robot_red)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:

        robot_blue.destroy_node()
        robot_red.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()