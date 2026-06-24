#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState, SpawnEntity, DeleteEntity
from geometry_msgs.msg import Wrench, Pose, Twist, Point, Quaternion
from std_msgs.msg import String, Bool
import math


# --- Seven-segment digit geometry (drawn in the world x-z plane, facing -y
#     toward the default camera). Segments are thin boxes; a digit is the set
#     of lit segments. The match ends at 5, so single digits (0-5) suffice. ---
SEGMENTS = {
    #      x      z     orientation
    'a': (0.0,  0.6, 'h'),   # top
    'b': (0.3,  0.3, 'v'),   # upper right
    'c': (0.3, -0.3, 'v'),   # lower right
    'd': (0.0, -0.6, 'h'),   # bottom
    'e': (-0.3, -0.3, 'v'),  # lower left
    'f': (-0.3, 0.3, 'v'),   # upper left
    'g': (0.0,  0.0, 'h'),   # middle
}
DIGITS = {
    0: 'abcdef',
    1: 'bc',
    2: 'abged',
    3: 'abgcd',
    4: 'fgbc',
    5: 'afgcd',
}
H_SIZE = '0.6 0.08 0.12'   # horizontal segment box (x y z)
V_SIZE = '0.12 0.08 0.6'   # vertical segment box


class BallController(Node):
    """Tracks the ball, scores goals, and runs a first-to-N match with an
    in-Gazebo seven-segment scoreboard.

    On each goal the score increments, the scoreboard digit for that team is
    respawned to the new number, the score is logged + published on
    /scoreboard, and the ball respawns to center. When either side reaches
    the winning score the match ends: a winner is announced and True is
    broadcast on /game_over (which the robot controllers watch to stop).
    """

    def __init__(self):
        super().__init__('ball_controller')

        self.ball_position = [0.0, 0.0, 0.0]

        # Field / goal geometry
        self.goal_width = 1.5     # goal opening half-width in y

        # --- Match state ---
        self.blue_score = 0
        self.red_score = 0
        self.win_score = 5
        self.game_over = False

        # Respawn cooldown to prevent rapid/double scoring
        self.respawn_cooldown = 0
        self.respawn_cooldown_max = 50  # 5 seconds at 10 Hz

        # Scoreboard model bookkeeping
        self.display_ready = False
        self.blue_model = None
        self.red_model = None

        # Gazebo service clients
        self.set_state_client = self.create_client(
            SetEntityState, '/gazebo/set_entity_state')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Publishers
        self.scoreboard_pub = self.create_publisher(String, '/scoreboard', 10)
        self.game_over_pub = self.create_publisher(Bool, '/game_over', 10)

        # Track the ball
        self.model_states_sub = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.model_states_callback, 10)

        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Ball controller initialized')
        self.publish_scoreboard()

    # ---------------------------------------------------------------
    # Ball tracking / scoring
    # ---------------------------------------------------------------
    def model_states_callback(self, msg):
        try:
            i = msg.name.index('football_ball')
            self.ball_position = [
                msg.pose[i].position.x,
                msg.pose[i].position.y,
                msg.pose[i].position.z,
            ]
            self.check_ball_out_of_bounds()
        except ValueError:
            self.get_logger().warn('Ball not found in model states', once=True)

    def check_ball_out_of_bounds(self):
        if self.game_over or self.respawn_cooldown > 0:
            return

        x = self.ball_position[0]
        y = self.ball_position[1]

        # Ball in left goal scores for Red, right goal scores for Blue.
        if x < -9.5 and abs(y) < self.goal_width:
            self.on_goal('red')
        elif x > 9.5 and abs(y) < self.goal_width:
            self.on_goal('blue')
        elif abs(x) > 10.0 or abs(y) > 5.0:
            self.get_logger().info('Ball out of bounds! Respawning to center.')
            self.reset_ball()
            self.respawn_cooldown = self.respawn_cooldown_max

    def on_goal(self, team):
        if team == 'blue':
            self.blue_score += 1
        else:
            self.red_score += 1

        self.publish_scoreboard(scored=team)
        self.update_digit(team)
        self.respawn_cooldown = self.respawn_cooldown_max

        if self.blue_score >= self.win_score or self.red_score >= self.win_score:
            self.end_game()
        else:
            self.reset_ball()
            self.reset_robots()

    def publish_scoreboard(self, scored=None):
        board = (
            f'SCORE  |  BLUE {self.blue_score}  -  {self.red_score} RED  '
            f'(first to {self.win_score})'
        )
        self.get_logger().info('=' * 52)
        if scored:
            self.get_logger().info(f'  GOAL!  {scored.upper()} scores!')
        self.get_logger().info(f'  {board}')
        self.get_logger().info('=' * 52)

        msg = String()
        msg.data = board
        self.scoreboard_pub.publish(msg)

    def end_game(self):
        self.game_over = True
        if self.blue_score > self.red_score:
            winner, hi, lo = 'BLUE', self.blue_score, self.red_score
        else:
            winner, hi, lo = 'RED', self.red_score, self.blue_score
        self.get_logger().info('#' * 52)
        self.get_logger().info(f'  GAME OVER!  {winner} WINS  {hi} - {lo}')
        self.get_logger().info('#' * 52)
        self.game_over_pub.publish(Bool(data=True))

    # ---------------------------------------------------------------
    # In-Gazebo scoreboard (seven-segment digits built from boxes)
    # ---------------------------------------------------------------
    @staticmethod
    def digit_sdf(number, rgba):
        r, g, b = rgba
        links = ''
        for seg in DIGITS[number]:
            x, z, orient = SEGMENTS[seg]
            size = H_SIZE if orient == 'h' else V_SIZE
            links += f'''
    <link name="seg_{seg}">
      <pose>{x} 0 {z} 0 0 0</pose>
      <visual name="v">
        <geometry><box><size>{size}</size></box></geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <emissive>{r * 0.5} {g * 0.5} {b * 0.5} 1</emissive>
        </material>
      </visual>
    </link>'''
        return (
            '<?xml version="1.0"?>\n'
            '<sdf version="1.6">\n'
            '  <model name="digit">\n'
            '    <static>true</static>'
            f'{links}\n'
            '  </model>\n'
            '</sdf>\n'
        )

    @staticmethod
    def box_sdf(rgba, size):
        r, g, b = rgba
        return (
            '<?xml version="1.0"?>\n'
            '<sdf version="1.6">\n'
            '  <model name="bar">\n'
            '    <static>true</static>\n'
            '    <link name="bar">\n'
            f'      <visual name="v"><geometry><box><size>{size}</size></box>'
            '</geometry>\n'
            f'        <material><ambient>{r} {g} {b} 1</ambient>'
            f'<diffuse>{r} {g} {b} 1</diffuse></material>\n'
            '      </visual>\n'
            '    </link>\n'
            '  </model>\n'
            '</sdf>\n'
        )

    def spawn_model(self, name, sdf, x, y, z):
        if not self.spawn_client.service_is_ready():
            return
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        req.reference_frame = 'world'
        self.spawn_client.call_async(req)

    def delete_model(self, name):
        if not name or not self.delete_client.service_is_ready():
            return
        req = DeleteEntity.Request()
        req.name = name
        self.delete_client.call_async(req)

    def init_display(self):
        # Separator dash between the two digits.
        self.spawn_model('score_sep', self.box_sdf((1, 1, 1), '0.4 0.08 0.12'),
                         0.0, 6.0, 2.8)
        self.update_digit('blue')
        self.update_digit('red')

    def update_digit(self, team):
        if team == 'blue':
            number, rgba, x = self.blue_score, (0.1, 0.3, 1.0), -1.5
            old = self.blue_model
        else:
            number, rgba, x = self.red_score, (1.0, 0.2, 0.2), 1.5
            old = self.red_model

        number = min(number, 5)
        new_name = f'score_{team}_{number}'
        # Spawn the new digit first (unique name), then remove the old one.
        self.spawn_model(new_name, self.digit_sdf(number, rgba), x, 6.0, 2.8)
        self.delete_model(old)

        if team == 'blue':
            self.blue_model = new_name
        else:
            self.red_model = new_name

    # ---------------------------------------------------------------
    # Ball reset
    # ---------------------------------------------------------------
    def set_entity_state(self, name, x, y, z, yaw=0.0):
        """Teleport a model to a pose with zero velocity."""
        if not self.set_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SetEntityState service not available')
            return

        req = SetEntityState.Request()
        req.state.name = name
        req.state.reference_frame = 'world'
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = float(z)
        req.state.pose.orientation.z = math.sin(yaw / 2.0)
        req.state.pose.orientation.w = math.cos(yaw / 2.0)
        # Zero velocity so nothing carries over from before the reset.
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        self.set_state_client.call_async(req)

    def reset_ball(self):
        self.get_logger().info('Resetting ball to center')
        self.set_entity_state('football_ball', 0.0, 0.0, 0.2)

    def reset_robots(self):
        """Return both robots to their kickoff spots (matches the launch file)."""
        self.get_logger().info('Resetting robots to start positions')
        self.set_entity_state('robot_blue', -3.0, 0.0, 0.51, 1.57)
        self.set_entity_state('robot_red', 3.0, 0.0, 0.51, -1.57)

    # ---------------------------------------------------------------
    # Main loop
    # ---------------------------------------------------------------
    def control_loop(self):
        # Spawn the scoreboard once the factory service is up.
        if not self.display_ready and self.spawn_client.service_is_ready():
            self.init_display()
            self.display_ready = True

        if self.respawn_cooldown > 0:
            self.respawn_cooldown -= 1

        # Continuously broadcast game-over so late/relaunched controllers sync.
        self.game_over_pub.publish(Bool(data=self.game_over))


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
