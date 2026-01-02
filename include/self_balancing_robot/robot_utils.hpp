#ifndef ROBOT_UTILS_HPP
#define ROBOT_UTILS_HPP

namespace robot_utils {
    void quaternion_to_euler(double x, double y, double z, double w, double &roll_x, double &pitch_y, double &yaw_z);
    void test();
}

#endif