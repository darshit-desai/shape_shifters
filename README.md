# Project ShapeShifters
![CICD Workflow status](https://github.com/shivamsehgal77/ShapeShifters/actions/workflows/run-unit-test-and-upload-codecov.yml/badge.svg) [![codecov](https://codecov.io/gh/shivamsehgal77/ShapeShifters/branch/main/graph/badge.svg)](https://codecov.io/gh/shivamsehgal77/ShapeShifters)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/license/mit/)

## Description
The project demonstrates a swarm control algorithm on a group of turtlebot robots. The turtlebot swarm executes a shape formation of different shapes when seen from the top view autonomously based on user input. The swarm will follow a leader follower configuration.

## Phase-1 Progress:
1. Design inital UML class and activity diagrams
2. Setup code coverage in codecov
3. Add launch files for launching a single turtlebot in empty world
4. Add template integration and unit tests

### Results
Below are the screenshots of UML and the launch file:

* Activity diagram

![](screenshots/Phase1_Results/UML_activity.png)

* Class diagram 

![](screenshots/Phase1_Results/UMLClassdiagram.png)

* Launch File run example

![](screenshots/Phase1_Results/launchfile_ex.png)


## Personnel
1. ![Darshit Desai](https://github.com/darshit-desai)
2. ![Patrik Pordi](https://github.com/patrikpordi)
3. ![Shivam Sehgal](https://github.com/shivamsehgal77)


## Build and Run Instructions
```bash
# Source to ROS2 HUMBLE
source /opt/ros/humble/setup.bash
# Make your ros2 workspace
mkdir ros2_ws
# Go to the source directory of your ros2 workspace
cd ~/ros2_ws
git clone https://github.com/shivamsehgal77/ShapeShifters.git
# Install process
## Ensure you have necessary packages installed
rosdep install -i --from-path src --rosdistro humble -y
sudo apt -y install ros-humble-desktop-full
sudo apt -y install ros-humble-gazebo-ros-pkgs
sudo apt -y install ros-humble-turtlebot3*
# Install rosdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
# Build the package using colcon build
colcon build --packages-select ShapeShifters --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --parallel-workers $(nproc)
# After successfull build source the package
. install/setup.bash
# ROS2 launch file run
export TURTLEBOT3_MODEL=burger && ros2 launch shape_shifters gazebo_launch.py 
```

## AIP and Product Backlog
1. Work Log and Sprint Log are at ![Link](https://docs.google.com/spreadsheets/d/1ph1sYep433EigfkVelYI8igBHbYIN74LMEw9CF0V7-I/edit#gid=2119248820)
2. Final Proposal Report ![Link](https://docs.google.com/document/d/1y6ZIHWb2QU3o7JkHODOUfTqYlNkz0YVxSIHP77x4x8U/edit?usp=sharing)

## CppCheck & CppLint
```bash
# Use the below command for cpp check by moving to root directory of your workspace
cppcheck --enable=all --std=c++17 --suppress=missingIncludeSystem $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) --check-config  &> cppcheck.txt

# Use the below command for cpp lint by moving to root directory of your workspace 
cpplint  --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $( find . -name *.cpp | grep -vE -e "^(./build/|./install/|./log/)" ) &> cpplint.txt 
```

## Test run instructions
```bash
colcon test --packages-select shape_shifters # Command to run the test
cat log/latest_test/shape_shifters/stdout_stderr.log # Command to view the test result
```

## Docs generation
```bash
sudo apt install -y doxygen lcov gcovr pandoc
colcon build --event-handlers console_cohesion+ --packages-select shape_shifters --cmake-target "docs"
```
