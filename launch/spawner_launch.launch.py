import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import numpy as np


def generate_launch_description():
    # Get the urdf file
    Number_of_Turtlebot3 = 24
    model_folder = "turtlebot3_" + "burger"
    urdf_path = os.path.join(
        get_package_share_directory("shape_shifters"),
        "models",
        model_folder,
        "model.sdf",
    )

    ld = LaunchDescription()

    # x_rand = np.random.uniform(-8, 8, Number_of_Turtlebot3)
    # y_rand = np.random.uniform(-8, 8, Number_of_Turtlebot3)
    # Number of robots
    num_robots = 24

    # Minimum distance between robots
    min_distance = 2.5

    # Size of the square field
    field_size = 15.0

    # Generate random initial positions with a minimum distance
    initial_positions = {}
    for i in range(0, num_robots):
        while True:
            # Generate random x and y coordinates within the field
            x = np.random.uniform(0, field_size)
            y = np.random.uniform(0, field_size)
            valid_position = all(
                np.linalg.norm(np.array([x, y]) - np.array(pos)) >= min_distance
                for pos in initial_positions.values()
            )

            if valid_position:
                initial_positions[f"turtlebot{i}"] = np.array([x, y])
                break
    positions_file_path = os.path.join(
        get_package_share_directory("shape_shifters"), "initial_positions.txt"
    )
    # export initial positions of robot to txt file
    with open(positions_file_path, "w") as file:
        for robot_name, position in initial_positions.items():
            x_pose = str(position[0])
            y_pose = str(position[1])
            file.write(f"{x_pose} {y_pose}\n")
    for i in range(Number_of_Turtlebot3):
        x_pose = str(initial_positions[f"turtlebot{i}"][0])
        y_pose = str(initial_positions[f"turtlebot{i}"][1])
        start_gazebo_ros_spawner_cmd = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            parameters=[{"verbose": True}],
            arguments=[
                "-entity",
                "robot_" + str(i),
                "-file",
                urdf_path,
                "-robot_namespace",
                "turtlebot3_" + str(i),
                "-x",
                x_pose,
                "-y",
                y_pose,
                "-z",
                "0.01",
            ],
            output="screen",
        )

        ld.add_action(
            TimerAction(
                period=9.0 + float(i * 2),
                actions=[start_gazebo_ros_spawner_cmd],
            )
        )

    return ld
