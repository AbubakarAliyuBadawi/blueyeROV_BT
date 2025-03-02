#include "gz_usbl/gz_usbl.h"

using namespace std::chrono_literals;

// Constructor parses ros parameters, sets the origin of the coordinate system, starts the odometry callback and starts the timed callback function
gz_sensors::GZ_USBL::GZ_USBL() : Node("gz_usbl") {

    // Get parameters/
    get_configuration_parameters();

    // If udp mode
    if (udp_mode_) {
        udp_client_.set_endpoint(port_, ip_address_);
    }
    else {
        usbl_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(usbl_topic_, 1);
    }

    // Start Odometry callback
    odometry_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
        odometry_topic_,
        1,
        bind(&GZ_USBL::odometry_callback, this, placeholders::_1)
    );

    // Start timer callback
    timer_ = create_wall_timer(750ms, bind(&GZ_USBL::usbl_callback, this));
}

gz_sensors::GZ_USBL::~GZ_USBL() {}

// Sets a new origin of the simulation based on user input
bool gz_sensors::GZ_USBL::set_new_origin(const float northing, const float easting) {
    
    // The northing should be larger than 0
    if (northing < 0.0) {
        return false;
    }
    // The easting should be larger than 0
    if (easting < 0.0) {
        return false;
    }

    // Set input to new origin
    origin_northing_ = northing;
    origin_easting_ = easting;
    return true;
}

// Set new sensor noise based on user input
bool gz_sensors::GZ_USBL::set_new_sensor_noise(const float sensor_noise) {

    // Check validity of sensor noise, it can not be negative
    if (sensor_noise < 0.0) {
        return false;
    }

    // Set new noise level based on input
    sensor_noise_ = sensor_noise;
    gauss_noise_generator_.set_parameters(0.0, sensor_noise);

    return true;
}

// Returns current northing
float gz_sensors::GZ_USBL::get_northing() {
    return origin_northing_;
}

// Returns current easting
float gz_sensors::GZ_USBL::get_easting() {
    return origin_easting_;
}

// Returns sensor noise
float gz_sensors::GZ_USBL::get_sensor_noise() {
    return sensor_noise_;
}

// Parses origin and usbl noise from ros parameters
void gz_sensors::GZ_USBL::get_configuration_parameters() {

    // Initialize usbl parameters
    gz_sensors::usbl_parameters params;

    // Declare ros parameters
    declare_parameter("gz_usbl_northing", 7037744.50);
    declare_parameter("gz_usbl_easting", 569442.50);
    declare_parameter("gz_usbl_sensor_noise", 0.3);
    declare_parameter("gz_usbl_wildpoint_probability", 0.001);
    declare_parameter("gz_usbl_wildpoint_max_size", 50.0);
    declare_parameter("gz_usbl_udp_mode", false);
    declare_parameter("gz_usbl_ip_address", "127.0.0.1");
    declare_parameter("gz_usbl_port", 8090);
    declare_parameter("gz_usbl_odometry_topic", "/rov100k/odometry_frd/gt");
    declare_parameter("gz_usbl_topic", "/rov100k/usbl");
    declare_parameter("gz_usbl_tp_code", "B12");
    declare_parameter("gz_usbl_pose", "-170 130 195.5");

    // Fetch ros parameters
    params.northing = get_parameter("gz_usbl_northing").as_double();
    params.easting = get_parameter("gz_usbl_easting").as_double();
    params.sensor_noise = get_parameter("gz_usbl_sensor_noise").as_double();
    params.wildpoint_probability = get_parameter("gz_usbl_wildpoint_probability").as_double();
    params.wildpoint_max_size = get_parameter("gz_usbl_wildpoint_max_size").as_double();
    params.udp_mode= get_parameter("gz_usbl_udp_mode").as_bool();
    params.ip_address = get_parameter("gz_usbl_ip_address").as_string();
    params.port = get_parameter("gz_usbl_port").as_int();
    params.odometry_topic = get_parameter("gz_usbl_odometry_topic").as_string();
    params.usbl_topic = get_parameter("gz_usbl_topic").as_string();
    params.tp_code = get_parameter("gz_usbl_tp_code").as_string();

    // Check validity of input
    if (!set_new_origin(params.northing, params.easting)) {
        RCLCPP_ERROR(get_logger(), "Northing and easting should be a float number larger than 0.0\
        \nGiven northing: %f\nGiven easting: %f", params.northing, params.easting); 
        exit(10);
    }

    if (!set_new_sensor_noise(params.sensor_noise)) {
        RCLCPP_ERROR(get_logger(), "Sensor noise should be a positive float number.\
        Given sensor noise: %f", params.sensor_noise);
        exit(10);
    }

    if (!utility_toolbox::ipv4_valid(params.ip_address)) {
        RCLCPP_ERROR(get_logger(), "The ip address provided is not a valid ipv4 address.\n\
        Provided address: %s", params.ip_address.c_str());
        exit(10);
    }

    if (params.port < 0) {
        RCLCPP_ERROR(get_logger(), "The port number is not valid. Provided port: %d", params.port);
        exit(10);
    }

    // Populaiting rest of class members
    udp_mode_ = params.udp_mode;
    ip_address_ = params.ip_address;
    port_ = params.port;
    odometry_topic_ = params.odometry_topic;
    usbl_topic_ = params.usbl_topic;
    auto position = get_parameter("gz_usbl_pose").as_string();
    x_position_ = stof(position.substr(0, position.find(" ")));
    y_position_ = stof(position.substr(position.find(" ") + 1, position.find(" ", position.find(" ") + 1)));
    z_position_ = stof(position.substr(position.find(" ", position.find(" ") + 1) + 1));

    return;
}

// Odometry callback
void gz_sensors::GZ_USBL::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // We have recived at least one odometry message
    if (!valid_odometry_) {
        valid_odometry_ = true;
    }

    // Retrives ground truth x, y and z position
    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;
    z_ = msg->pose.pose.position.z;
}


// Generates a PSIMGPS message
string gz_sensors::GZ_USBL::get_psimgps() {
    /* desired format 
    start characther - $
    address - PSIMGPS
    data time - yymmddhhmmss.ss 
    utm or geo - U or G
    north coordinate - float
    north or south - N or S
    east coordinate - float
    east or west - E or W
    checksum - checksum
    termination - CRLF
    */

    // Intialize string
    string s = "$PSIMGPS,";
    
    // time field
    auto now = chrono::system_clock::now();
    auto time_now = chrono::system_clock::to_time_t(now);
    auto tm_now = *localtime(&time_now);
    ostringstream oss;
    oss << put_time(&tm_now, "%y%m%d%H%M%S");
    auto milliseconds = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    oss << '.' << std::setw(2) << std::setfill('0') << milliseconds.count()/10;
    string time_str =  oss.str();
    s += time_str + ",";

    // utm or geo field
    s += "G,";

    // north coordinate:
    s += to_string(abs(origin_northing_)) + ",";

    // north or south 
    if (x_ >= 0.0) {
        s += "N,";
    }
    else {
        s += "S,";
    }

    // east coordinate
    s += to_string(abs(origin_easting_)) + ",";

    // east or west
    if (abs(y_) > 0.0) {
        s += "E,";
    }
    else {
        s += "W,";
    }

    // checksum
    s += "3,3,701.276";

    // termination
    s += "\r\n";

    return s;
}

// Generate a PSIMSSB message 
string gz_sensors::GZ_USBL::get_psimssb() {
    /*desired format
    start charatcher - $
    address - PSIMSSB
    time - hhmmss.ss
    tp-code - transponder code
    status - A for OK V for not OK
    error code - empty or code
    coordinate system - C for cartesian, P for polar, U for utm
    orientation - H for vesel head up, N for north E for east
    SW_filter - M measured, F filtered, P predicted
    x coordinate - depends on apos config
    y coordinate - depends on apos config
    depth - depth in meters
    expected accuracy - float
    addition info - N none, I inclinometer, D depth, T time
    first add value - based on prev
    second add value - based on prev
    checksum - 
    termination - CRLF
    */

    // Initialize string
    string s = "$PSIMSSB,";

    // time field
    auto now = chrono::system_clock::now();
    auto time_now = chrono::system_clock::to_time_t(now);
    auto tm_now = *localtime(&time_now);
    ostringstream oss;
    oss << put_time(&tm_now, "%H%M%S");
    auto milliseconds = chrono::duration_cast<chrono::milliseconds>(now.time_since_epoch()) % 1000;
    oss << '.' << std::setw(2) << std::setfill('0') << milliseconds.count()/10;
    string time_str =  oss.str();
    s += time_str + ",";

    // tp code
    s += tp_code_ + ",";

    // status
    s += "A,";

    // error code
    s += ",";

    // coordinate system
    s += "U,";

    // Orientation
    s += "N,";

    // sw filter
    s += "M,";


    // x_coordinate && y_coordinate
    double noisy_x = origin_northing_ + x_ + gauss_noise_generator_.generate_noise();
    double noisy_y = origin_easting_ + y_ + gauss_noise_generator_.generate_noise();

    // Check for wildpoints
    if (probability_sampler_.sample() <= wildpoint_probability_) {
        if (probability_sampler_.sample() > 0.5) {
            noisy_x += probability_sampler_.sample() * wildpoint_max_size_;
        }
        else {
            noisy_x -= probability_sampler_.sample() * wildpoint_max_size_;
        }
        if (probability_sampler_.sample() > 0.5) {
            noisy_y += probability_sampler_.sample() * wildpoint_max_size_;
        }
        else {
            noisy_y -= probability_sampler_.sample() * wildpoint_max_size_;
        }
    }

    s += to_string(noisy_x) + ",";
    s += to_string(noisy_y) + ",";

    // depth
    s += to_string(z_) + ",";

    // accuracy
    s += "1.173,";

    // Extra
    s += "N,";

    // Ending
    s += ",\r\n";

    return s;
}

// USBL callback
void gz_sensors::GZ_USBL::usbl_callback() {
    
    // Checks that we have recived at least on valid odometry measurement
    if (valid_odometry_) {

        // Are we publishing to a ros topic or using udp?
        if (!udp_mode_) {
            auto msg = pose_stamped_msgs();
            usbl_publisher_->publish(msg);
        }
        else {
            string psimssb = get_psimssb();
            udp_client_.send_message(psimssb);
        }
    }
}


// Generate POSE message
geometry_msgs::msg::PoseStamped gz_sensors::GZ_USBL::pose_stamped_msgs() {
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = x_ - x_position_;
    pose_stamped.pose.position.y = y_ - y_position_;
    pose_stamped.pose.position.z = z_ - z_position_;

    // Angle between docking station and robot
    float delta_x = x_ - x_position_;
    float delta_y = y_ - y_position_;
    float angle = atan2(delta_y, delta_x);

    // Convert yaw angle to quaternion
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, angle); // Roll and pitch are zero
    pose_stamped.pose.orientation.x = quaternion.x();
    pose_stamped.pose.orientation.y = quaternion.y();
    pose_stamped.pose.orientation.z = quaternion.z();
    pose_stamped.pose.orientation.w = quaternion.w();

    return pose_stamped;
}
