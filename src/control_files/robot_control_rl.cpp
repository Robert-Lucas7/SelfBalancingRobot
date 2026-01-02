
#include "self_balancing_robot/robot_utils.hpp"
// rclcpp is included in robot_control_rl.hpp below.
#include "self_balancing_robot/robot_control_rl.hpp"

RobotControlRL::RobotControlRL() : Node("robot_control_rl") {
    RCLCPP_INFO(this->get_logger(), "RobotControlRL node has been started.");
    
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlRL>());
    rclcpp::shutdown();
    return 0;
}