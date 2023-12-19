import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
import numpy as np

def generate_launch_description():
    # Your TurtleBot model (e.g., 'burger' or 'waffle')
    turtlebot_model = 'burger'

    # Number of TurtleBots to spawn
    num_turtlebots = 20

    # Spacing between TurtleBots along the x-axis
    spacing_x = 1.0

    # Launch file directory
    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )

    # Include the Gazebo empty world launch
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(launch_file_dir, 'empty_world.launch.py')
        ])
    )

    # Generate random positions for TurtleBots within a reasonable range
    x_rand = np.random.uniform(-5, 5, num_turtlebots)
    y_rand = np.random.uniform(-5, 5, num_turtlebots)

    # Create nodes to spawn multiple TurtleBots
    spawn_turtlebots = []
    for i in range(num_turtlebots):
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            namespace=f'turtlebot{i}',
            arguments=[
                '-entity', f'turtlebot{i}',
                '-file', f'/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_{turtlebot_model}/model.sdf',
                '-x', str(x_rand[i]),  # Adjust the x pose
                '-y', str(y_rand[i]),  # Adjust the y pose
                '-z', '0.01'
            ],
        )
        spawn_turtlebots.append(node)

        # Add TimerAction for introducing a delay before each spawn
        timer_action = TimerAction(
            period=10.0 + float(i * 2),  # Adjust the delay period
            actions=[node],
        )
        ld.add_action(timer_action)

    ld = LaunchDescription([
        empty_world_launch,
        *spawn_turtlebots
    ])

    # Print information for debugging
    print("Number of TurtleBots:", num_turtlebots)
    print("Spawn positions (x):", x_rand)
    print("Spawn positions (y):", y_rand)

    return ld
