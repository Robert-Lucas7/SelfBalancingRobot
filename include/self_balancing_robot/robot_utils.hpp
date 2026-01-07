#ifndef ROBOT_UTILS_HPP
#define ROBOT_UTILS_HPP

#include <cmath>
#include "angles/angles.h"

namespace robot_utils {
    template<typename T>
    void quaternion_to_euler(T x, T y, T z, T w, T &roll_x, T &pitch_y, T &yaw_z){
        // Roll (x-axis rotation)
        T t0 = 2.0 * (w * x + y * z);
        T t1 = 1.0 - 2.0 * (x * x + y * y);
        roll_x = std::atan2(t0, t1);

        // Pitch (y-axis rotation)
        T t2 = 2.0 * (w * y - z * x);
        if (t2 > 1.0) t2 = 1.0;
        if (t2 < -1.0) t2 = -1.0;
        pitch_y = std::asin(t2);

        // Yaw (z-axis rotation)
        double t3 = 2.0 * (w * z + x * y);
        double t4 = 1.0 - 2.0 * (y * y + z * z);
        yaw_z = std::atan2(t3, t4);
    }

    template<typename T>
    bool checkEpisodeFinished(T pitch) {
        if(std::abs(pitch) > angles::from_degrees(45.0)){
            return true;
        } else {
            return false;
        }
    }

    
}

#endif