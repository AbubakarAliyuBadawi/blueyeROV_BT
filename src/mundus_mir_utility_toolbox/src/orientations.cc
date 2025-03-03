#include "utility_toolbox/orientations.h"

// Calculates the samllest signed angle assuming the coordinate system goes from -PI to PI
double utility_toolbox::smallest_signed_angle(const double desired_angle, const double current_angle) {

    // Checks if angle is larger than PI, if it is do calculation based on sign
    double error = desired_angle - current_angle;
    if (abs(error) > M_PI) {
        return error > 0 ? error - 2*M_PI : error + 2*M_PI;
    }
    return error;
}

// Returns an Euler_Angle struct given the individual elements of an quaternion
utility_toolbox::Euler_Angles utility_toolbox::quat_to_euler(const double q_w, const double q_x, const double q_y, const double q_z) {

    // From wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    utility_toolbox::Euler_Angles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q_w * q_x + q_y * q_z);
    double cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q_w * q_y - q_x * q_z));
    double cosp = sqrt(1 - 2 * (q_w * q_y - q_x * q_z));
    angles.pitch = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q_w * q_z + q_x * q_y);
    double cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

// Returns a Quaternion struct given the roll, pitch and yaw angle
utility_toolbox::Quaternion euler_to_quat(const double roll, const double pitch, const double yaw) {

    // From wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    utility_toolbox::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}