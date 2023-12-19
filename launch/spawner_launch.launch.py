import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import numpy as np

def generate_launch_description():
    # Get the urdf file
    Number_of_Turtlebot3 = 20
    model_folder = 'turtlebot3_' + 'burger'
    urdf_path = os.path.join(
        get_package_share_directory('shape_shifters'),
        'models',
        model_folder,
        'model.sdf'
    )

    ld = LaunchDescription()

    x_rand = np.random.uniform(-8, 8, Number_of_Turtlebot3)
    y_rand = np.random.uniform(-8, 8, Number_of_Turtlebot3)

    for i in range(Number_of_Turtlebot3):
        x_pose = str(x_rand[i])
        y_pose = str(y_rand[i])
        start_gazebo_ros_spawner_cmd = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            parameters=[{'verbose': True}],
            arguments=[
                '-entity', 'Tb'+str(i),
                '-file', urdf_path,
                "-robot_namespace", 'turtlebot3_'+str(i),
                '-x', x_pose,
                '-y', y_pose,
                '-z', '0.01'
            ],
            output='screen',
        )

        ld.add_action(TimerAction(period=9.0+float(i*2), actions=[start_gazebo_ros_spawner_cmd],))
 

    return ld