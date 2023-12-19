import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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

    x_rand = np.random.uniform(-10, 10, num_turtlebots)
    y_rand = np.random.uniform(-10, 10, num_turtlebots)
    # Create nodes to spawn multiple TurtleBots
    spawn_turtlebots = [
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-entity', f'turtlebot{i}',
                "-robot_namespace", f"tb{i}",
                '-file', f'/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf',
                '-x', str(x_rand[i]),  # Adjust the x pose
                '-y', str(y_rand[i]),  # Adjust the y pose
                '-z', '0.01'
            ],
        ) for i in range(num_turtlebots)
    ]

    return LaunchDescription([
        empty_world_launch,
        *spawn_turtlebots
    ])
