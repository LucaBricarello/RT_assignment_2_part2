# Requirements

 - ROS2 foxy
 - package robot_urdf (https://github.com/CarmineD8/robot_urdf.git)
 - Gazebo

# How to run the code
After sourcing the right setup files, to run the code you simply need to clone this repository inside your ROS2 workspace, compile it with colcon build, then run the following commands inside the ws folder:

	- ros2 launch robot_urdf gazebo.launch.py
To run the simulation.
And the in a new terminal:

	- ros2 run assignment_2_part2 move_node
To run the node developed in this package

# move_node
move_node it's a node that with the use of a subscriber to a the /odom topic gets the information about the position and uses it to compute the velocities to make the robot follow a straight/curvilinear path decided in the code.

The velocities are published on the cmd_vel to make the robot move.
