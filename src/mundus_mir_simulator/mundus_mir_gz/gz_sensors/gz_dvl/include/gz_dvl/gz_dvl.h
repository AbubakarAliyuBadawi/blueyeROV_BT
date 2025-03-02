#ifndef GZ_DVL_H_
#define GZ_DVL_H_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <marine_acoustic_msgs/msg/dvl.hpp>
#include <string>
#include <cmath>

#include "utility_toolbox/network.h"
#include "utility_toolbox/distributions.h"

using namespace std;

namespace gz_sensors {

class GZ_DVL : public rclcpp::Node {

    // Methods
    public:
        GZ_DVL();
        ~GZ_DVL();

        // Helper functions, should be service callable
        bool set_velocity_sensor_noise(const float sensor_noise);
        bool set_altitude_sensor_noise(const float sensor_noise);
        bool set_wildpoint_probability(const float wildpoint_probability);
        float get_velocity_sensor_noise();
        float get_altitude_sensor_noise();
        float get_wildpoint_probability();

    // Methods
    private:
        void get_configuration_parameters();
        void dvl_body_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void range_to_bottom_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void dvl_callback();
        string get_pd6_message();

    // Members
    private:
        // Parameters in configuration file
        string dvl_topic_, dvl_body_topic_, range_topic_, ip_address_;
        float max_valid_altitude_, publish_frequency_, altitude_sensor_noise_, velocity_sensor_noise_, wildpoint_probability_;
        bool udp_mode_;
        int port_;

        // DVL message containers
        bool signal_valid_ = false;
        double vel_x_ = -9999;
        double vel_y_ = -9999;
        double vel_z_ = -9999;
        float altitude_ = -9999;

        // Subscribers and publishers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_body_subscriber_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr range_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<marine_acoustic_msgs::msg::Dvl>::SharedPtr dvl_publisher_;

        // Simple UDP client
        utility_toolbox::Simple_UDP_Client udp_client_;

        // Gaussian noise generator
        utility_toolbox::Gaussian_Noise velocity_gauss_noise_generator_;
        utility_toolbox::Gaussian_Noise altitude_gauss_noise_generator_;

        // Wildpoint generator
        utility_toolbox::Sample_Probability probability_sampler_;

};
};

#endif