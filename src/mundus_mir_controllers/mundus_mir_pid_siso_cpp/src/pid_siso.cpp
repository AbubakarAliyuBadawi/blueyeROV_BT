#include "pid_siso.hpp"

double PID_siso::compute (double setPointProportional,
                          double currentValueProportional,
                          double setPointDerivative,
                          double currentValueDerivative,
                          double feedForward) {

    // Proportional term
    auto errorProportional = setPointProportional - currentValueProportional;

    // Smallest-signed angle correction
    if (ssaOn_) {
        errorProportional = ssa (errorProportional);
    }

    // Derivative term
    auto errorDerivative = setPointDerivative - currentValueDerivative;

    // Integral term
    integralTerm_ += errorProportional;

    // Compute PID output
    auto p = kp_ * errorProportional;
    auto i = ki_ * integralTerm_;
    auto d = kd_ * errorDerivative;

    auto output = p + i + d + feedForward;

    // Saturation limits check
    if (output <= saturationLower_) {
        return saturationLower_;
    } else if (output >= saturationUpper_) {
        return saturationUpper_;
    } else {
        return output;
    }
};

void PID_siso::resetIntegralToZero () {
    integralTerm_ = 0.0;
};

void PID_siso::setParameters (double kp,
                              double ki,
                              double kd,
                              double saturationUpper,
                              double saturationLower,
                              bool ssaOn) {
    kp_              = kp;
    ki_              = ki;
    kd_              = kd;
    saturationUpper_ = saturationUpper;
    saturationLower_ = saturationLower;
    ssaOn_            = ssaOn;
};

double PID_siso::ssa (const double angle) const {
    // returns the smallest-signed angle in [ -pi, pi )
    return std::fmod ((angle + M_PI), (2 * M_PI)) - M_PI;
};
