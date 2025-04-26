#include "gz_battery/gz_battery.h"
#include <chrono>
#include <random>

using namespace std::chrono_literals;

namespace gz_sensors {

GZ_Battery::GZ_Battery() : Node("gz_battery"), state_of_charge_(1.0f) { // Start at 100% SOC
    // Fetch ROS parameters
    get_configuration_parameters();

    // Battery status publisher
    battery_publisher_ = create_publisher<mundus_mir_msgs::msg::BatteryStatus>(battery_topic_, 1);

    // Subscribe to thruster velocity commands
    for (size_t i = 0; i < 4; ++i) {
        std::string topic_name = "/blueye/thruster_" + std::to_string(i + 1) + "/cmd_vel";
        thruster_subscriptions_.emplace_back(
            create_subscription<std_msgs::msg::Float64>(
                topic_name, 10,
                [this, i](std_msgs::msg::Float64::SharedPtr msg) {
                    thruster_cmds_[i] = msg->data;
                }));
    }

    // Set up timer based on the publish frequency
    auto time = std::chrono::milliseconds(static_cast<int>(1000 / publish_frequency_));
    timer_ = create_wall_timer(time, std::bind(&GZ_Battery::battery_callback, this));

    // Seed the random number generator
    rng_.seed(std::random_device()());
}

GZ_Battery::~GZ_Battery() {}

void GZ_Battery::get_configuration_parameters() {

    // Declare and fetch ROS parameters
    declare_parameter("battery_topic", "blueye/battery");
    declare_parameter("publish_frequency", 1.0);
    declare_parameter("voltage_sensor_noise_mean", 0.0);
    declare_parameter("voltage_sensor_noise_stddev", 0.1);
    declare_parameter("temperature_sensor_noise_mean", 0.0);
    declare_parameter("temperature_sensor_noise_stddev", 0.1);
    declare_parameter("thruster_power_coefficient", 0.05);
    
    // Fetch parameters
    battery_topic_ = get_parameter("battery_topic").as_string();
    publish_frequency_ = get_parameter("publish_frequency").as_double();
    voltage_sensor_noise_mean_ = get_parameter("voltage_sensor_noise_mean").as_double();
    voltage_sensor_noise_stddev_ = get_parameter("voltage_sensor_noise_stddev").as_double();
    temperature_sensor_noise_mean_ = get_parameter("temperature_sensor_noise_mean").as_double();
    temperature_sensor_noise_stddev_ = get_parameter("temperature_sensor_noise_stddev").as_double();
    thruster_power_coefficient_ = get_parameter("thruster_power_coefficient").as_double();


    // Initialize noise generators with parameters
    voltage_gauss_noise_generator_.set_parameters(voltage_sensor_noise_mean_, voltage_sensor_noise_stddev_);
    temperature_gauss_noise_generator_.set_parameters(temperature_sensor_noise_mean_, temperature_sensor_noise_stddev_);

    // Initialize thruster commands to zero
    thruster_cmds_.fill(0.0);
}

void GZ_Battery::battery_callback() {
    // Generate battery status with values influenced by thruster commands
    auto message = mundus_mir_msgs::msg::BatteryStatus();

    // Simulate total power consumption based on thruster commands
    double total_power_consumption = 0.0;
    for (double cmd : thruster_cmds_) {
        total_power_consumption += thruster_power_coefficient_ * std::abs(cmd);
    }

    // Normalize power consumption to slow depletion
    const double depletion_factor = 2000.0; // Adjust this factor for desired runtime
    const double normalized_power = total_power_consumption / depletion_factor;

    // Adjust persistent state of charge (SOC)
    state_of_charge_ = std::max(0.0f, state_of_charge_ - static_cast<float>(normalized_power));

    // Populate message with current SOC and related values
    message.state_of_charge = state_of_charge_;
    message.remaining_capacity = message.full_charge_capacity * state_of_charge_;

    // Generate other fields with noise
    message.total_voltage = random_float(14.0, 15.5) + voltage_gauss_noise_generator_.generate_noise();
    message.average_temperature = random_float(10.0, 15.0) + temperature_gauss_noise_generator_.generate_noise();
    message.initialization = true;
    message.error_status = "BATTERY_ERROR_OK";
    message.current = total_power_consumption;
    message.average_current = total_power_consumption;
    message.runtime_to_empty = (total_power_consumption > 0.0)
                                   ? static_cast<int>(3600 * state_of_charge_ / normalized_power)
                                   : -1; // -1 indicates infinite runtime

    // Publish the updated battery status
    battery_publisher_->publish(message);
    // RCLCPP_INFO(this->get_logger(),
    //             "Published battery status (SOC: %.2f%%, Power: %.2fW, Depletion factor: %.2f)",
    //             state_of_charge_ * 100, total_power_consumption, depletion_factor);
}


float GZ_Battery::random_float(float min, float max) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng_);
}

int GZ_Battery::random_int(int min, int max) {
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng_);
}

}  // namespace gz_sensors
