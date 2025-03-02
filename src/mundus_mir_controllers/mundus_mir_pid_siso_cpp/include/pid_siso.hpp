#pragma once

// Single input single output pid controller. The controller includes support
// for constant feed forward and integral reset. The controller does not have
// flags for activating and deactivating the integral action.

#include <math.h>
#include <string>
#include <vector>

class PID_siso {
    public:
    PID_siso ()
    : kp_ (0.0), ki_ (0.0), kd_ (0.0), saturationUpper_ (0.0),
      saturationLower_ (0.0), ssaOn_(false){};

    PID_siso (double kp, double ki, double kd, double saturationUpper, double saturationLower, bool ssaOn)
    : kp_ (kp), ki_ (ki), kd_ (kd), saturationUpper_ (saturationUpper),
      saturationLower_ (saturationLower), ssaOn_(ssaOn){};

    double ssa (const double angle) const;

    double compute (double setPointProportional,
                    double currentValueProportional,
                    double setPointDerivative,
                    double currentValueDerivative,
                    double feedForward);

    void resetIntegralToZero ();

    void setParameters (double kp, double ki, double kd, double saturationUpper, double saturationLower, bool ssaOn);

    private:
    double kp_, ki_, kd_;
    double integralTerm_;
    double saturationUpper_, saturationLower_;
    bool ssaOn_;
};