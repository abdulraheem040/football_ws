import os
import subprocess
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def process_xacro_to_file(xacro_file):
    """Process xacro file and write to temporary URDF file"""
    try:
        result = subprocess.run(
            ['xacro', xacro_file],
            capture_output=True,
            text=True,
            check=True
        )
        # Write to temporary file
        temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
        temp_file.write(result.stdout)
        temp_file.close()
        return temp_file.name
    except subprocess.CalledProcessError as e:
        print(f"Error processing xacro file: {e}")
        print(f"stderr: {e.stderr}")
        return None


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('humanoid_football')
    
    # Paths
    world_file = os.path.join(pkg_dir, 'worlds', 'football_field.world')
    robot_blue_xacro = os.path.join(pkg_dir, 'urdf', 'robot_blue.xacro')
    robot_red_xacro = os.path.join(pkg_dir, 'urdf', 'robot_red.xacro')
    ball_urdf = os.path.join(pkg_dir, 'urdf', 'football_ball.urdf')

    # Process xacro files to temporary URDF files
    robot_blue_urdf_file = process_xacro_to_file(robot_blue_xacro)
    robot_red_urdf_file = process_xacro_to_file(robot_red_xacro)
    
    if robot_blue_urdf_file is None or robot_red_urdf_file is None:
        raise RuntimeError("Failed to process xacro files")

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set gazebo verbose mode'
    )

    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': LaunchConfiguration('verbose'),
        }.items()
    )

    # Gazebo client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]
        )
    )

    # Spawn robot blue with delay to ensure Gazebo is ready
    spawn_robot_blue = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot_blue',
                    '-file', robot_blue_urdf_file,
                    '-x', '-3.0',
                    '-y', '0.0',
                    '-z', '0.45',
                    '-Y', '1.57'
                ],
                output='screen'
            )
        ]
    )

    # Spawn robot red with delay
    spawn_robot_red = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'robot_red',
                    '-file', robot_red_urdf_file,
                    '-x', '3.0',
                    '-y', '0.0',
                    '-z', '0.45',
                    '-Y', '-1.57'
                ],
                output='screen'
            )
        ]
    )

    # Spawn football ball with delay
    spawn_ball = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'football_ball',
                    '-file', ball_urdf,
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.2'
                ],
                output='screen'
            )
        ]
    )

    # Robot controller node with delay
    robot_controller = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='humanoid_football',
                executable='robot_controller',
                output='screen'
            )
        ]
    )

    # Ball controller node with delay
    ball_controller = TimerAction(
        period=7.5,
        actions=[
            Node(
                package='humanoid_football',
                executable='ball_controller',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        world_arg,
        verbose_arg,
        gazebo_server,
        gazebo_client,
        spawn_robot_blue,
        spawn_robot_red,
        spawn_ball,
        robot_controller,
        ball_controller,
    ])
