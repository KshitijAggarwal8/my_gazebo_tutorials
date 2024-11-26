# my_gazebo_tutorials
Kshitij Aggarwal

119211618

# Overview:

A basic ROS2 package that implements obstacle avoidance TurtleBot using State design Pattern.

NOTE: This package assumes that TURTLEBOT package is already installed in any of the other ROS2 overlays on the system.

# Build instructions

1. Create a workspace, with the package "walker" in the src/
2. Move to the base of the workspace.
3. Check for dependencies: rosdep install -i --from-path src --rosdistro humble -y
4. Build the package: colcon build 

# Gazebo launch instructions:
1. Source setup files: source install/setup.bash
2. export TURTLEBOT3_MODEL=burger
3. ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py


# Obstacle avoidance launch instruction:
1. Open a new terminal in the same directory.
2. source install/setup.bash
3. Launch the OA: ros2 launch walker walker.launch.py record_bag:=true

arguments:

a. record_bag - For enabling/disabling rosbag recording from the launch file.



# Record and play ROSBAG:

To execute the ros_bag recorder, simply pass the argument "enable_rosbag" as "true".

To replay the rosbag, run: ros2 bag play "name_of_your_rosbag_found_in_the_root_directory"

NOTE: A sample rosbag can be found in `results/`

