#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool
from rclpy.executors import MultiThreadedExecutor
import math
import random


def normalize_angle(a):
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def lerp(a, b, t):
    return a + (b - a) * t


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class RobotController(Node):
    """Autonomous striker: get behind the ball, then drive it into the
    OPPONENT's goal (never its own).

    All poses come from the world-frame /gazebo/model_states topic so the
    geometry is consistent. Velocities are slew-rate limited for smooth,
    stable motion. The robot detects when it is stuck and performs an escape
    maneuver, and freezes when ball_controller announces game over.
    """

    def __init__(self, robot_name='robot'):
        super().__init__(f'{robot_name}_controller')

        self.robot_name = robot_name
        self.opponent_name = (
            'robot_red' if robot_name == 'robot_blue' else 'robot_blue'
        )

        # World-frame pose, filled from /gazebo/model_states
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.have_pose = False

        self.ball_x = 0.0
        self.ball_y = 0.0
        self.have_ball = False

        self.opponent_x = 0.0
        self.opponent_y = 0.0
        self.opponent_detected = False

        # Remember BOTH goals. Attack the opponent's goal; protect, and never
        # shoot at, the own goal. Must agree with ball_controller scoring:
        #   ball in right goal (x > 9.5)  -> Blue scores
        #   ball in left  goal (x < -9.5) -> Red scores
        if robot_name == 'robot_blue':
            self.target_goal_x = 10.0    # opponent goal (right)
            self.own_goal_x = -10.0      # own goal (left)
        else:
            self.target_goal_x = -10.0   # opponent goal (left)
            self.own_goal_x = 10.0       # own goal (right)
        self.target_goal_y = 0.0

        # --- Per-robot "personality" (random; the two robots play
        #     differently each run). Speeds kept moderate for stability. ---
        self.aggression = random.uniform(0.0, 1.0)
        self.base_linear = lerp(1.1, 2.0, self.aggression)
        self.base_angular = lerp(2.2, 3.4, self.aggression)
        self.base_strike = lerp(1.8, 3.0, self.aggression)
        self.approach_offset = lerp(0.70, 0.45, self.aggression)

        # Tunables
        self.strike_distance = 1.2
        self.strike_align = 0.35

        # --- Dynamic pacing ---
        self.burst_mult = 1.0
        self.burst_ticks = 0
        self.kick_power = self.base_strike
        self.mode = 'seek'
        self.seek_ticks = 0      # how long we've been chasing without a strike
        self.charge_ticks = 0    # active "charge the ball" burst

        # --- Smoothing: slew-rate limit published velocities ---
        self.cmd_lin = 0.0
        self.cmd_ang = 0.0
        self.max_lin_step = 0.12
        self.max_ang_step = 0.50   # snappier turning so it can change direction

        # --- Stuck detection / escape ---
        self.stuck_timer = 0
        self.stuck_ref_x = 0.0
        self.stuck_ref_y = 0.0
        self.stuck_ref_set = False
        self.win_intended = False     # did we try to move during this window?
        self.stuck_count = 0
        self.escape_ticks = 0
        self.escape_turn = 1.0

        # Game state
        self.game_over = False

        # Publisher / subscribers
        self.cmd_vel_pub = self.create_publisher(
            Twist, f'/{robot_name}/cmd_vel', 10
        )
        self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10
        )
        self.create_subscription(Bool, '/game_over', self.game_over_callback, 10)

        self.create_timer(0.05, self.control_loop)   # 20 Hz

        self._log_tick = 0
        self.get_logger().info(
            f'{robot_name}: attack goal x={self.target_goal_x:.0f}, '
            f'defend goal x={self.own_goal_x:.0f} | aggression {self.aggression:.2f}'
        )

    # ---------------------------------------------------------------
    # State input
    # ---------------------------------------------------------------
    @staticmethod
    def yaw_from_quat(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def game_over_callback(self, msg):
        if msg.data and not self.game_over:
            self.game_over = True
            self.get_logger().info(f'{self.robot_name}: game over - stopping.')

    def model_states_callback(self, msg):
        names = msg.name
        try:
            i = names.index(self.robot_name)
            p = msg.pose[i]
            self.current_x = p.position.x
            self.current_y = p.position.y
            self.current_yaw = self.yaw_from_quat(p.orientation)
            self.have_pose = True
        except ValueError:
            pass
        try:
            i = names.index('football_ball')
            self.ball_x = msg.pose[i].position.x
            self.ball_y = msg.pose[i].position.y
            self.have_ball = True
        except ValueError:
            pass
        try:
            i = names.index(self.opponent_name)
            self.opponent_x = msg.pose[i].position.x
            self.opponent_y = msg.pose[i].position.y
            self.opponent_detected = True
        except ValueError:
            self.opponent_detected = False

    def bearing_to(self, x, y):
        """Relative bearing (rad) from the robot's heading to world point (x, y)."""
        return normalize_angle(
            math.atan2(y - self.current_y, x - self.current_x) - self.current_yaw
        )

    def publish_smoothed(self, target_lin, target_ang):
        """Move current command toward the target by a bounded step, then publish."""
        self.cmd_lin += clamp(target_lin - self.cmd_lin, -self.max_lin_step, self.max_lin_step)
        self.cmd_ang += clamp(target_ang - self.cmd_ang, -self.max_ang_step, self.max_ang_step)
        twist = Twist()
        twist.linear.x = self.cmd_lin
        twist.angular.z = self.cmd_ang
        self.cmd_vel_pub.publish(twist)

    # ---------------------------------------------------------------
    # Control
    # ---------------------------------------------------------------
    def control_loop(self):
        # Game over: glide to a stop.
        if self.game_over:
            self.publish_smoothed(0.0, 0.0)
            return

        # Escape maneuver in progress: back up while turning to break free.
        if self.escape_ticks > 0:
            self.escape_ticks -= 1
            self.publish_smoothed(-0.5, self.escape_turn)
            return

        # No data yet: spin and creep to look around.
        if not self.have_pose or not self.have_ball:
            self.publish_smoothed(0.1, 1.2)
            return

        # Re-roll a sprint/ease multiplier periodically.
        self.burst_ticks -= 1
        if self.burst_ticks <= 0:
            self.burst_mult = random.uniform(0.85, 1.35)
            self.burst_ticks = random.randint(20, 60)

        bx, by = self.ball_x, self.ball_y
        gx, gy = self.target_goal_x, self.target_goal_y

        dist_ball = math.hypot(bx - self.current_x, by - self.current_y)

        # Stand-off point sits behind the ball, away from the opponent goal,
        # so driving forward from it sends the ball goalward.
        ball_to_goal = math.atan2(gy - by, gx - bx)
        approach_x = bx - math.cos(ball_to_goal) * self.approach_offset
        approach_y = by - math.sin(ball_to_goal) * self.approach_offset

        robot_to_ball = math.atan2(by - self.current_y, bx - self.current_x)
        alignment = normalize_angle(robot_to_ball - ball_to_goal)

        bearing_ball = self.bearing_to(bx, by)
        aligned = abs(alignment) < self.strike_align
        close = dist_ball < self.strike_distance

        # Situational speed shaping.
        if dist_ball > 3.0:
            pace = 1.25
        elif dist_ball > 1.5:
            pace = 1.0
        else:
            pace = 0.8
        chase = 1.0
        if self.opponent_detected:
            d_opp_ball = math.hypot(bx - self.opponent_x, by - self.opponent_y)
            chase = 1.15 if dist_ball <= d_opp_ball else 0.9

        lin = self.base_linear * self.burst_mult * pace * chase
        ang = self.base_angular * self.burst_mult

        if aligned and close and abs(bearing_ball) < 0.6:
            # STRIKE: drive through the ball toward the opponent goal.
            if self.mode != 'strike':
                self.kick_power = self.base_strike * random.uniform(0.9, 1.3)
                self.mode = 'strike'
            self.seek_ticks = 0
            self.charge_ticks = 0
            target_lin = self.kick_power
            target_ang = clamp(ang * bearing_ball, -1.5, 1.5)
            self._log(f'STRIKE dist={dist_ball:.2f} power={self.kick_power:.2f}')
        else:
            # SEEK: pursue the ball, then line up behind it.
            self.mode = 'seek'
            self.seek_ticks += 1

            # Unable to strike for a long time: charge the ball directly to
            # shake it loose and change its direction.
            if self.seek_ticks > 120 and self.charge_ticks == 0:
                self.charge_ticks = 25
                self.seek_ticks = 0
                self._log('charging the ball to change its direction')

            if self.charge_ticks > 0:
                self.charge_ticks -= 1
                tgt_x, tgt_y, charging = bx, by, True
            elif dist_ball > 2.0:
                # Far away: head straight for the ball to close the gap fast.
                tgt_x, tgt_y, charging = bx, by, False
            else:
                # Close: aim for the stand-off point behind the ball.
                tgt_x, tgt_y, charging = approach_x, approach_y, False

            bearing_t = self.bearing_to(tgt_x, tgt_y)
            target_ang = clamp(ang * bearing_t, -ang, ang)
            target_lin = lin if abs(bearing_t) < 0.6 else 0.2
            if charging:
                target_lin = self.base_strike
            elif dist_ball < 0.6:
                # Don't shove the ball while circling behind it (own-goal guard).
                target_lin = min(target_lin, 0.12)
            self._log(f'SEEK dist={dist_ball:.2f} lin={target_lin:.2f}')

        # Opponent avoidance: steer away and ease off so they don't ram.
        if self.opponent_detected:
            d_opp = math.hypot(
                self.opponent_x - self.current_x, self.opponent_y - self.current_y
            )
            bearing_opp = self.bearing_to(self.opponent_x, self.opponent_y)
            if d_opp < 1.3 and abs(bearing_opp) < 0.8:
                target_ang += -1.4 if bearing_opp >= 0 else 1.4
                target_lin = min(target_lin, 0.4)

        self.update_stuck_detection(target_lin)
        self.publish_smoothed(target_lin, target_ang)

    def update_stuck_detection(self, target_lin):
        """If we keep commanding motion but barely move, trigger an escape."""
        if abs(target_lin) > 0.25:
            self.win_intended = True

        if not self.stuck_ref_set:
            self.stuck_ref_x = self.current_x
            self.stuck_ref_y = self.current_y
            self.stuck_ref_set = True

        self.stuck_timer += 1
        if self.stuck_timer >= 10:   # check every ~0.5 s
            moved = math.hypot(
                self.current_x - self.stuck_ref_x,
                self.current_y - self.stuck_ref_y,
            )
            if self.win_intended and moved < 0.1:
                self.stuck_count += 1
            else:
                self.stuck_count = 0

            if self.stuck_count >= 2:   # ~1 s of trying but not moving
                self.escape_ticks = 30  # ~1.5 s back-up-and-turn
                self.escape_turn = random.choice([-1.0, 1.0]) * 1.6
                self.stuck_count = 0
                self.get_logger().info(f'{self.robot_name}: stuck - escaping.')

            self.stuck_ref_x = self.current_x
            self.stuck_ref_y = self.current_y
            self.win_intended = False
            self.stuck_timer = 0

    def _log(self, text):
        self._log_tick += 1
        if self._log_tick % 40 == 0:
            self.get_logger().info(f'{self.robot_name}: {text}')


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
