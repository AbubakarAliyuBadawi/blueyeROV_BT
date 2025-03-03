#ifndef ORIENTATIONS_H_
#define ORIENTATIONS_H_

#include <math.h>

namespace utility_toolbox {

struct Euler_Angles {
    double roll, pitch, yaw;
};

struct Quaternion {
    double w, x, y, z;
};

// Get smallest signed angle between a desired and current angle assuming a coordinate system going from -Pi to Pi
double smallest_signed_angle(const double desired_angle, const double current_angle);

// Returns an Euler_Angle struct given the individual elements of an quaternion
Euler_Angles quat_to_euler(const double q_w, const double q_x, const double q_y, const double q_z);

// Returns a Quaternion struct given the roll, pitch and yaw angle
Quaternion euler_to_quat(const double roll, const double pitch, const double yaw);

}

#endif