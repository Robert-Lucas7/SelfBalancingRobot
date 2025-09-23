To build the package use:
colcon build --packages-select self_balancing_robot

Then source the install with:
source ./install/local_setup.bash

Then launch gazebo and the robot controller with the following command:
ros2 launch self_balancing_robot <launch file selected>

Available launch files:
- pid_control - The robot is controlled using a PID controller with the pitch of the body as the input.

