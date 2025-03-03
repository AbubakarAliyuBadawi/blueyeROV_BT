#include "utility_toolbox/distributions.h"


///////////////////////////////////////////// Gausian_Noise ///////////////////////////////////////////////

utility_toolbox::Gaussian_Noise::Gaussian_Noise(const double mean, const double variance) {

    // Set the current time as seed for our random engine
    auto seed = chrono::system_clock::now().time_since_epoch().count();
    engine_.seed(seed);

    // Update distribution based on input
    dist_ptr_ = make_unique<normal_distribution<double>>(mean, sqrt(variance));
}

utility_toolbox::Gaussian_Noise::~Gaussian_Noise() {};

// Set new mean and variance for the distribution
void utility_toolbox::Gaussian_Noise::set_parameters(const double mean, const double variance) {
    dist_ptr_ = make_unique<normal_distribution<double>>(mean, sqrt(variance));
}

// 
double utility_toolbox::Gaussian_Noise::generate_noise() {
    return (*dist_ptr_)(engine_);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

utility_toolbox::Sample_Probability::Sample_Probability() {

    // Set the current time as seed for our random engine
    auto seed = chrono::system_clock::now().time_since_epoch().count();
    engine_.seed(seed);

    dist_ptr_ = make_unique<uniform_real_distribution<>>(0.0, 1.0);
}

utility_toolbox::Sample_Probability::~Sample_Probability() {};

double utility_toolbox::Sample_Probability::sample() {
    return (*dist_ptr_)(engine_);
}



/////////////////////////////////////////// Sample Probability ////////////////////////////////////////////
