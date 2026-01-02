#include "self_balancing_robot/robot_utils.hpp"
#include <cmath>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace robot_utils {
    void quaternion_to_euler(double x, double y, double z, double w, double &roll_x, double &pitch_y, double &yaw_z){
    // Roll (x-axis rotation)
    double t0 = 2.0 * (w * x + y * z);
    double t1 = 1.0 - 2.0 * (x * x + y * y);
    roll_x = std::atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = 2.0 * (w * y - z * x);
    if (t2 > 1.0) t2 = 1.0;
    if (t2 < -1.0) t2 = -1.0;
    pitch_y = std::asin(t2);

    // Yaw (z-axis rotation)
    double t3 = 2.0 * (w * z + x * y);
    double t4 = 1.0 - 2.0 * (y * y + z * z);
    yaw_z = std::atan2(t3, t4);
  }

  void test() {
    auto logger = rclcpp::get_logger("robot_utils");
    RCLCPP_INFO(logger, "TESTING robot_utils.cpp");
  }
}