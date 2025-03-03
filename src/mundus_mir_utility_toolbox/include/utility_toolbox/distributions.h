#ifndef DISTRIBUTIONS_H_
#define DISTRIBUTIONS_H_

#include <iostream>
#include <random>
#include <memory>
#include <cmath>
#include <chrono>

using namespace std;

namespace utility_toolbox {

// Simple class to generate gaussian noise based on a given mean and variance
class Gaussian_Noise {

    public:
        // Constructors 
        Gaussian_Noise(const double mean = 0.0, const double variance = 1.0);
        ~Gaussian_Noise();

        // Set new mean and variance
        void set_parameters(const double mean, const double variance);

        // Generate noise based on gaussian distribution
        double generate_noise();

    private:
        default_random_engine engine_;
        unique_ptr<normal_distribution<double>> dist_ptr_;

};

// Simple class to get a random number between 0 and 1
class Sample_Probability {

    public:
        // Constructors
        Sample_Probability();
        ~Sample_Probability();

        // Sample
        double sample();

    private:
        default_random_engine engine_;
        unique_ptr<uniform_real_distribution<>> dist_ptr_;

};

};

#endif