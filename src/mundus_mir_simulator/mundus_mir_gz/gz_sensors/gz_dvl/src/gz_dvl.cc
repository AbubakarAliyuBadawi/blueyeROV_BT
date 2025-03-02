#include "gz_dvl/gz_dvl.h"

gz_sensors::GZ_DVL::GZ_DVL() : Node("gz_dvl") {

    // Fetch ros parameters
    get_configuration_parameters();

    // Odometry subscriber
    dvl_body_subscriber_= create_subscription<nav_msgs::msg::Odometry>(
        dvl_body_topic_,
        1,
        bind(&GZ_DVL::dvl_body_callback, this, placeholders::_1)
    );

    // Range subscriber
    range_subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        range_topic_,
        1,
        bind(&GZ_DVL::range_to_bottom_callback, this, placeholders::_1)
    );

    // Check for UDP mode
    if (udp_mode_) {
        udp_client_.set_endpoint(port_, ip_address_);
    }
    else {
        dvl_publisher_ = create_publisher<marine_acoustic_msgs::msg::Dvl>(dvl_topic_, 1);
    }

    // Start dvl callback
    auto time = 1000ms/publish_frequency_;
    timer_ = create_wall_timer(time, bind(&GZ_DVL::dvl_callback, this));

};

gz_sensors::GZ_DVL::~GZ_DVL() {};

void gz_sensors::GZ_DVL::get_configuration_parameters() {

    // Declaring ROS parameters
    declare_parameter("dvl_topic", "rov100k/dvl_full");
    declare_parameter("dvl_body_topic", "rov100k/dvl");
    declare_parameter("range_topic", "rov100k/dvl/ray");
    declare_parameter("max_valid_altitude", 30.0);
    declare_parameter("publish_frequency", 10.0);
    declare_parameter("udp_mode", false);
    declare_parameter("port", 8091);
    declare_parameter("ip_address", "127.0.0.1");
    declare_parameter("velocity_sensor_noise", 0.001);
    declare_parameter("altitude_sensor_noise", 0.001);
    declare_parameter("wildpoint_probability", 0.001);

    // Fetch parameters
    dvl_topic_ = get_parameter("dvl_topic").as_string();
    dvl_body_topic_ = get_parameter("dvl_body_topic").as_string();
    range_topic_ = get_parameter("range_topic").as_string();
    max_valid_altitude_ = get_parameter("max_valid_altitude").as_double();
    publish_frequency_ = get_parameter("publish_frequency").as_double();
    udp_mode_ = get_parameter("udp_mode").as_bool();
    port_ = get_parameter("port").as_int();
    ip_address_ = get_parameter("ip_address").as_string();
    velocity_sensor_noise_ = get_parameter("velocity_sensor_noise").as_double();
    altitude_sensor_noise_ = get_parameter("altitude_sensor_noise").as_double();
    wildpoint_probability_ = get_parameter("wildpoint_probability").as_double();

    // Set sensor noise
    set_velocity_sensor_noise(velocity_sensor_noise_);
    set_altitude_sensor_noise(altitude_sensor_noise_);

};

void gz_sensors::GZ_DVL::dvl_body_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Update velocities with noise
    vel_x_ = msg->twist.twist.linear.x + velocity_gauss_noise_generator_.generate_noise();
    vel_y_ = msg->twist.twist.linear.y + velocity_gauss_noise_generator_.generate_noise();
    vel_z_ = msg->twist.twist.linear.z + velocity_gauss_noise_generator_.generate_noise();

};

// The altimeter for Gazebo Garden dosen't work. Therfore a small depth camera is used instead to measure the altitude of the DVL.
void gz_sensors::GZ_DVL::range_to_bottom_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    // Getting range to bottom
    float range_to_bottom = msg->ranges[0];

    if (range_to_bottom < max_valid_altitude_) {
        altitude_ = range_to_bottom + altitude_gauss_noise_generator_.generate_noise();
        signal_valid_ = true;
    }
    else {
        signal_valid_ = false;
    }

};

void gz_sensors::GZ_DVL::dvl_callback() {

    // Check that velocity values are recived
    if (vel_x_ != -9999) {

    // Checks for UDP mode
    if (udp_mode_) {
        string packet = get_pd6_message();
        udp_client_.send_message(packet);
    }
    else {
        
        // Declaring message container
        marine_acoustic_msgs::msg::Dvl msg;
        std_msgs::msg::Header header;

        // Sett up header
        header.frame_id = "body";
        header.stamp = now();
        msg.header = header;

        // Payload
        msg.sound_speed = 1500.0;
        msg.altitude = altitude_;
        msg.beam_ranges_valid = signal_valid_;
        msg.beam_velocities_valid = signal_valid_;
        msg.altitude = altitude_;
        geometry_msgs::msg::Vector3 vel;
        vel.x = vel_x_;
        vel.y = vel_y_;
        vel.z = vel_z_;
        msg.velocity = vel;
        msg.velocity_mode = 1;
        msg.course_gnd = atan2(vel_y_, vel_x_);
        msg.speed_gnd = sqrt(vel_x_*vel_x_ + vel_y_*vel_y_);

        // Publish message
        dvl_publisher_->publish(msg);
    }
    }
};

bool gz_sensors::GZ_DVL::set_velocity_sensor_noise(const float sensor_noise) {

    // Check validity of sensor noise, it can not be negative
    if (sensor_noise < 0.0) {
        return false;
    }

    // Set new noise level based on input
    velocity_sensor_noise_ = sensor_noise;
    velocity_gauss_noise_generator_.set_parameters(0.0, sensor_noise);
    return true;
}

bool gz_sensors::GZ_DVL::set_altitude_sensor_noise(const float sensor_noise) {

    // Check validity of sensor noise, it can not be negative
    if (sensor_noise < 0.0) {
        return false;
    }

    // Set new noise level based on input
    altitude_sensor_noise_ = sensor_noise;
    altitude_gauss_noise_generator_.set_parameters(0.0, sensor_noise);
    return true;
}

bool gz_sensors::GZ_DVL::set_wildpoint_probability(const float wildpoint_probability) {

    // Probability should be between 0.0 and 1.0
    if (wildpoint_probability < 0.0 || wildpoint_probability > 1.0) {
        return false;
    }

    wildpoint_probability_ = wildpoint_probability;
    return true;
}

float gz_sensors::GZ_DVL::get_velocity_sensor_noise() {
    return velocity_sensor_noise_;
}

float gz_sensors::GZ_DVL::get_altitude_sensor_noise() {
    return altitude_sensor_noise_;
}

float gz_sensors::GZ_DVL::get_wildpoint_probability() {
    return wildpoint_probability_;
}

string gz_sensors::GZ_DVL::get_pd6_message() {
    // TODO - Find current format of Workhorse DVL
    return "Hello World!";
};