#include "rclcpp/rclcpp.hpp"
#include "self_balancing_robot/robot_utils.hpp"

class RobotControlRL: public rclcpp::Node {
    public:
        RobotControlRL(): Node("robot_control_rl") {
            RCLCPP_INFO(this->get_logger(), "RobotControlRL node has been started.");
            robot_utils::test();
        }
    private:
        // Private members
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotControlRL>());
    rclcpp::shutdown();
    return 0;
}