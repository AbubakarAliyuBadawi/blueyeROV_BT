#ifndef GZ_USBL_H
#define GZ_USBL_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <chrono>
#include <iomanip>
#include <tf2/LinearMath/Quaternion.h>

#include "utility_toolbox/network.h"
#include "utility_toolbox/distributions.h"

using namespace std;


namespace gz_sensors {

struct usbl_parameters {
    float northing;
    float easting;
    float sensor_noise;
    float wildpoint_probability;
    float wildpoint_max_size;
    bool udp_mode;
    string ip_address;
    int port;
    string odometry_topic;
    string usbl_topic;
    string tp_code;
};

class GZ_USBL : public rclcpp::Node {

    // Methods
    public:
        // Constructors
        GZ_USBL();
        ~GZ_USBL();

        // Helper functions, should be service callable
        bool set_new_origin(const float northing, const float easting);
        bool set_new_sensor_noise(const float sensor_noise);
        float get_northing();
        float get_easting();
        float get_sensor_noise();

    // Methods
    private:
        // Update member variables with data based on cofniguration file
        void get_configuration_parameters();
        
        // Timed callback that publishes the NMEA string containing the easting and northing of the vehicle
        void usbl_callback();
        void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Generate different nmea strings
        string get_psimgps();
        string get_psimssb();
        
        // Generate POSE message
        geometry_msgs::msg::PoseStamped pose_stamped_msgs();

    // Members
    private:
        // Parameters in configuration file
        float origin_northing_, origin_easting_, sensor_noise_, wildpoint_probability_, wildpoint_max_size_;
        bool udp_mode_;
        int port_;
        string ip_address_, odometry_topic_, usbl_topic_, tp_code_;
        float x_position_, y_position_, z_position_;


        // Odometry message parameters
        float x_, y_, z_;
        bool valid_odometry_ = false;

        // Ros parameters
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr usbl_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;

        // Simple UDP client
        utility_toolbox::Simple_UDP_Client udp_client_;

        // Gaussian noise generator
        utility_toolbox::Gaussian_Noise gauss_noise_generator_;

        // Wildpoint generator
        utility_toolbox::Sample_Probability probability_sampler_;
        
};
};
#endif