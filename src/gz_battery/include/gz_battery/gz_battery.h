#ifndef GZ_BATTERY__GZ_BATTERY_H_
#define GZ_BATTERY__GZ_BATTERY_H_

#include "rclcpp/rclcpp.hpp"
#include "mundus_mir_msgs/msg/battery_status.hpp"
#include "std_msgs/msg/float64.hpp"
#include <array>
#include <random>

namespace gz_sensors {

class GZ_Battery : public rclcpp::Node
{
public:
    GZ_Battery();
    ~GZ_Battery();

private:
    // ROS 2 components
    rclcpp::Publisher<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr battery_publisher_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> thruster_subscriptions_;
    rclcpp::TimerBase::SharedPtr timer_;
    

    // Parameters
    std::string battery_topic_;
    double publish_frequency_;
    double voltage_sensor_noise_mean_;
    double voltage_sensor_noise_stddev_;
    double temperature_sensor_noise_mean_;
    double temperature_sensor_noise_stddev_;
    double thruster_power_coefficient_; // Power coefficient for thrusters

    // Thruster command values
    std::array<double, 4> thruster_cmds_; // Array to store thruster command values

    float state_of_charge_; // Persistent state of charge

    // Noise generators
    std::mt19937 rng_; // Random number generator
    class GaussianNoiseGenerator {
    public:
        GaussianNoiseGenerator() : dist_(0.0, 1.0) {}
        void set_parameters(double mean, double stddev) {
            dist_ = std::normal_distribution<double>(mean, stddev);
        }
        double generate_noise() {
            return dist_(rng_);
        }
    private:
        std::mt19937 rng_;
        std::normal_distribution<double> dist_;
    };
    GaussianNoiseGenerator voltage_gauss_noise_generator_;
    GaussianNoiseGenerator temperature_gauss_noise_generator_;

    // Methods
    void get_configuration_parameters();
    void battery_callback();
    float random_float(float min, float max);
    int random_int(int min, int max);
};

}  // namespace gz_sensors

#endif  // GZ_BATTERY__GZ_BATTERY_H_

