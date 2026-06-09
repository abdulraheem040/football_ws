# Humanoid Football ROS 2 Project

A ROS 2 project featuring two humanoid robots playing football in Gazebo simulation.

## Features

- Two humanoid robots (Blue and Red teams)
- Football ball with realistic physics
- Gazebo world with a football field including goals
- Robot control nodes for autonomous football playing
- Goal detection and ball tracking

## Prerequisites

- ROS 2 Humble (or compatible version)
- Gazebo 11+
- Python 3
- Required ROS 2 packages:
  - gazebo_ros
  - xacro
  - robot_state_publisher
  - joint_state_publisher
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - gazebo_msgs

## Installation

1. Navigate to your ROS 2 workspace:
```bash
cd ~/football_ws
```

2. Build the workspace:
```bash
colcon build
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Launch the Simulation

To launch the complete football simulation with both robots and the ball:

```bash
ros2 launch humanoid_football football_simulation.launch.py
```

This will:
- Start Gazebo with the football field world
- Spawn the blue robot at position (-3.0, 0.0)
- Spawn the red robot at position (3.0, 0.0)
- Spawn the football ball at the center (0.0, 0.0)
- Start robot state publishers for both robots
- Start joint state publisher

### Run Robot Controllers

In a separate terminal, source the workspace and run the robot controller:

```bash
source install/setup.bash
ros2 run humanoid_football robot_controller
```

This will start autonomous control for both robots to:
- Detect the ball
- Move towards the ball
- Kick the ball towards the opponent's goal

### Run Ball Controller

In another terminal, run the ball controller to track goals:

```bash
source install/setup.bash
ros2 run humanoid_football ball_controller
```

## Project Structure

```
humanoid_football/
├── launch/
│   └── football_simulation.launch.py    # Main launch file
├── urdf/
│   ├── humanoid_robot.xacro              # Base humanoid robot model
│   ├── robot_blue.xacro                  # Blue team robot
│   ├── robot_red.xacro                   # Red team robot
│   └── football_ball.urdf                # Football ball model
├── worlds/
│   └── football_field.world              # Gazebo world with field
├── humanoid_football/
│   ├── robot_controller.py               # Robot control node
│   └── ball_controller.py                # Ball/goal tracking node
├── package.xml
├── setup.py
└── setup.cfg
```

## Robot Model

The humanoid robot includes:
- Torso (base link)
- Head
- Two arms (upper and lower with shoulder and elbow joints)
- Two legs (upper and lower with hip, knee, and ankle joints)
- Feet

The robot uses a differential drive controller for locomotion via the hip joints.

## Football Field

The Gazebo world includes:
- Green ground plane
- White field markings (boundary, center line, center circle)
- Goal areas and penalty areas
- Two goals with posts, crossbars, and nets
- Proper lighting and shadows

## Control Logic

The robot controller implements a simple football playing strategy:
1. Detect the ball position (simulated)
2. Rotate towards the ball
3. Move towards the ball
4. When close to the ball, rotate towards the opponent's goal
5. Kick the ball forward

The ball controller:
- Tracks the ball position from Gazebo model states
- Detects when a goal is scored
- Logs goal events

## Customization

### Modify Robot Appearance

Edit the color properties in `urdf/robot_blue.xacro` and `urdf/robot_red.xacro`:
```xml
<xacro:macro name="robot_color" params="blue"/>  <!-- or "red" -->
<xacro:macro name="skin_color" params="white"/>
```

### Modify Field Dimensions

Edit the field dimensions in `worlds/football_field.world` to change the field size, goal positions, etc.

### Adjust Control Parameters

Modify the control parameters in `humanoid_football/robot_controller.py`:
- Movement speeds
- Detection thresholds
- Goal positions

## Troubleshooting

### Gazebo doesn't launch
- Ensure Gazebo is properly installed: `gazebo --version`
- Check that gazebo_ros package is installed

### Robots don't spawn
- Verify URDF files are correctly formatted
- Check that xacro is installed: `ros2 run xacro xacro --help`

### Controllers don't work
- Ensure topics are correctly named
- Check that robot plugins are loaded in Gazebo
- Verify topic names match between launch file and controller

## Future Enhancements

- Add camera sensors for real ball detection
- Implement more sophisticated football strategies
- Add multiple robots per team
- Implement referee logic
- Add score tracking
- Add keyboard/teleop control for manual robot control

## License

MIT License

## Author

abdul_raheem
