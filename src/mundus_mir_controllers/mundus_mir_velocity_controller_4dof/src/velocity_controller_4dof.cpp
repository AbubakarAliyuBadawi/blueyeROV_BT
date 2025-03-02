#include "velocity_controller_4dof.hpp"

VelocityController4dof::VelocityController4dof ()
: Node ("velocity_controller_4dof"), surge_param_ (6), sway_param_ (6),
  depth_param_ (6), yaw_param_ (6) {

    // ROS2 Parameters
    // Parameter: ROS2 QoS buffer size
    declare_parameter<int> ("qos_buffer_size", 10);
    get_parameter ("qos_buffer_size", param_qos_buffer_size_);

    // Parameter: Current values subscriber topic name
    declare_parameter<std::string> ("topic_subscriber_odometry", "sub_odom_current");
    get_parameter ("topic_subscriber_odometry", param_topic_sub_odometry_);

    // Parameter: Desired values subscriber topic name
    declare_parameter<std::string> ("topic_subscriber_desired", "sub_odom_desired");
    get_parameter ("topic_subscriber_desired", param_topic_sub_desired_);

    // Parameter: Publisher topic name
    declare_parameter<std::string> ("topic_publisher", "pub_velocity_controller_wrench");
    get_parameter ("topic_publisher", param_topic_pub_);

    // Parameter: Set controller callback frequency in Hz
    declare_parameter<int> ("controller_frequency_hz", 100);
    get_parameter ("controller_frequency_hz", param_controller_frequency_);

    // Parameters Gain Values and Saturation limits: kp, ki,
    // kd, feedForward, satUpper, satLower.
    declare_parameter<double> ("rt_surge_kp", 0.0);
    declare_parameter<double> ("rt_surge_ki", 0.0);
    declare_parameter<double> ("rt_surge_kd", 0.0);
    declare_parameter<double> ("rt_surge_feedforward", 0.0);
    declare_parameter<double> ("rt_surge_satUpper", 10.0);
    declare_parameter<double> ("rt_surge_satLower", -10.0);
    declare_parameter<double> ("rt_sway_kp", 0.0);
    declare_parameter<double> ("rt_sway_ki", 0.0);
    declare_parameter<double> ("rt_sway_kd", 0.0);
    declare_parameter<double> ("rt_sway_feedforward", 0.0);
    declare_parameter<double> ("rt_sway_satUpper", 10.0);
    declare_parameter<double> ("rt_sway_satLower", -10.0);
    declare_parameter<double> ("rt_depth_kp", 0.0);
    declare_parameter<double> ("rt_depth_ki", 0.0);
    declare_parameter<double> ("rt_depth_kd", 0.0);
    declare_parameter<double> ("rt_depth_feedforward", 0.0);
    declare_parameter<double> ("rt_depth_satUpper", 10.0);
    declare_parameter<double> ("rt_depth_satLower", -10.0);
    declare_parameter<double> ("rt_yaw_kp", 0.0);
    declare_parameter<double> ("rt_yaw_ki", 0.0);
    declare_parameter<double> ("rt_yaw_kd", 0.0);
    declare_parameter<double> ("rt_yaw_feedforward", 0.0);
    declare_parameter<double> ("rt_yaw_satUpper", 500.0);
    declare_parameter<double> ("rt_yaw_satLower", -500.0);

    get_parameter ("rt_surge_kp", surge_param_ (0));
    get_parameter ("rt_surge_ki", surge_param_ (1));
    get_parameter ("rt_surge_kd", surge_param_ (2));
    get_parameter ("rt_surge_feedforward", surge_param_ (3));
    get_parameter ("rt_surge_satUpper", surge_param_ (4));
    get_parameter ("rt_surge_satLower", surge_param_ (5));
    get_parameter ("rt_sway_kp", sway_param_ (0));
    get_parameter ("rt_sway_ki", sway_param_ (1));
    get_parameter ("rt_sway_kd", sway_param_ (2));
    get_parameter ("rt_sway_feedforward", sway_param_ (3));
    get_parameter ("rt_sway_satUpper", sway_param_ (4));
    get_parameter ("rt_sway_satLower", sway_param_ (5));
    get_parameter ("rt_depth_kp", depth_param_ (0));
    get_parameter ("rt_depth_ki", depth_param_ (1));
    get_parameter ("rt_depth_kd", depth_param_ (2));
    get_parameter ("rt_depth_feedforward", depth_param_ (3));
    get_parameter ("rt_depth_satUpper", depth_param_ (4));
    get_parameter ("rt_depth_satLower", depth_param_ (5));
    get_parameter ("rt_yaw_kp", yaw_param_ (0));
    get_parameter ("rt_yaw_ki", yaw_param_ (1));
    get_parameter ("rt_yaw_kd", yaw_param_ (2));
    get_parameter ("rt_yaw_feedforward", yaw_param_ (3));
    get_parameter ("rt_yaw_satUpper", yaw_param_ (4));
    get_parameter ("rt_yaw_satLower", yaw_param_ (5));


    // Initialize controllers parameters
    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    surgeController_.setParameters (surge_param_ (0), surge_param_ (1), surge_param_ (2),
                                    surge_param_ (4), surge_param_ (5), false);
    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    swayController_.setParameters (sway_param_ (0), sway_param_ (1), sway_param_ (2),
                                   sway_param_ (4), sway_param_ (5), false);

    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    depthController_.setParameters (depth_param_ (0), depth_param_ (1), depth_param_ (2),
                                    depth_param_ (4), depth_param_ (5), false);

    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    yawController_.setParameters (yaw_param_ (0), yaw_param_ (1), yaw_param_ (2),
                                  yaw_param_ (4), yaw_param_ (5), true);

    // ROS2 Subscribers and Publishers
    subscriber_odometry_ = create_subscription<nav_msgs::msg::Odometry> (
    param_topic_sub_odometry_, param_qos_buffer_size_,
    std::bind (&VelocityController4dof::subscriber_odometry_callback_, this,
               std::placeholders::_1));

    subscriber_desired_ = create_subscription<geometry_msgs::msg::TwistStamped> (
    param_topic_sub_desired_, param_qos_buffer_size_,
    std::bind (&VelocityController4dof::subscriber_desired_callback_, this,
               std::placeholders::_1));

    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped> (
    param_topic_pub_, param_qos_buffer_size_);


    // ROS2 Timer
    double freq_hz    = param_controller_frequency_;
    double period_sec = 1.0 / freq_hz;
    std::chrono::nanoseconds period_ns (static_cast<long long> (period_sec * 1e9));
    timer_ = create_wall_timer (
    period_ns, std::bind (&VelocityController4dof::pid_callback_, this));

    // Populate initial values for current and desired
    current_values_ << 0.0, 0.0, 0.0, 0.0;
    desired_values_ << 0.0, 0.0, 0.0, 0.0;

    // ROS2 Parameter callback
    params_callback_handle_ = add_on_set_parameters_callback (
    std::bind (&VelocityController4dof::paramsCallback, this, std::placeholders::_1));
}

void VelocityController4dof::subscriber_odometry_callback_ (
const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Callback for receiving the current values msg and populating object
    // variables.
    auto vel_x   = msg->twist.twist.linear.x;
    auto vel_y   = msg->twist.twist.linear.y;
    auto vel_z   = msg->twist.twist.linear.z;
    auto vel_psi = msg->twist.twist.angular.z;

    current_values_ << vel_x, vel_y, vel_z, vel_psi;
}

void VelocityController4dof::subscriber_desired_callback_ (
const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // Callback for receiving the desired values msg and populating object
    // variables.
    auto vel_x   = msg->twist.linear.x;
    auto vel_y   = msg->twist.linear.y;
    auto vel_z   = msg->twist.linear.z;
    auto vel_psi = msg->twist.angular.z;

    desired_values_ << vel_x, vel_y, vel_z, vel_psi;
}

void VelocityController4dof::pid_callback_ () {
    // Compute pid
    auto x = surgeController_.compute (desired_values_ (0), current_values_ (0),
                                       0.0, 0.0, surge_param_ (3));

    auto y = swayController_.compute (desired_values_ (1), current_values_ (1),
                                      0.0, 0.0, sway_param_ (3));

    auto z = depthController_.compute (desired_values_ (2), current_values_ (2),
                                       0.0, 0.0, depth_param_ (3));

    auto psi = yawController_.compute (desired_values_ (3), current_values_ (3),
                                       0.0, 0.0, yaw_param_ (3));

    // Fetch timestamp
    auto now = std::chrono::system_clock::now ();
    auto now_s =
    std::chrono::time_point_cast<std::chrono::seconds> (now).time_since_epoch ();
    auto now_ns =
    std::chrono::time_point_cast<std::chrono::nanoseconds> (now).time_since_epoch ();
    auto now_ns_fraction =
    now_ns - std::chrono::duration_cast<std::chrono::seconds> (now_ns);

    int now_s_c  = now_s.count ();
    int now_ns_c = now_ns_fraction.count ();

    // Write Wrench message
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp.nanosec = now_ns_c;
    msg.header.stamp.sec     = now_s_c;
    msg.wrench.force.x       = x;
    msg.wrench.force.y       = y;
    msg.wrench.force.z       = z;
    msg.wrench.torque.z      = psi;

    // Publish message
    publisher_->publish (msg);
}

rcl_interfaces::msg::SetParametersResult VelocityController4dof::paramsCallback (
const std::vector<rclcpp::Parameter>& params) {

    // Declare return value
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason     = "Successful!";

    for (const auto& param : params) {
        if (param.get_name () == "qos_buffer_size") {
            if (param.as_string () == "") {
                result.reason     = "Parameter string cannot be empty.";
                result.successful = false;
            } else {
                param_qos_buffer_size_ = param.as_int ();
                result.reason          = "Parameter cannot change at run-time.";
            }
        }
        if (param.get_name () == "topic_subscriber_odometry") {
            if (param.as_string () == "") {
                result.reason     = "Parameter string cannot be empty.";
                result.successful = false;
            } else {
                param_topic_sub_odometry_ = param.as_string ();
                result.reason = "Parameter cannot change at run-time.";
            }
        }
        if (param.get_name () == "topic_subscriber_desired") {
            if (param.as_string () == "") {
                result.reason     = "Parameter string cannot be empty.";
                result.successful = false;
            } else {
                param_topic_sub_desired_ = param.as_string ();
                result.reason = "Parameter cannot change at run-time.";
            }
        }
        if (param.get_name () == "topic_publisher") {
            if (param.as_string () == "") {
                result.reason     = "Parameter string cannot be empty.";
                result.successful = false;
            } else {
                param_topic_pub_ = param.as_string ();
                result.reason    = "Parameter cannot change at run-time";
            }
        }
        if (param.get_name () == "controller_frequency_hz") {
            if (param.as_int () < 0) {
                result.reason     = "Parameter cannot be a negative integer";
                result.successful = false;
            } else {
                param_controller_frequency_ = param.as_int ();
                result.reason = "Parameter cannot change at run-time.";
            }
        }
        if (param.get_name () == "rt_surge_kp") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                surge_param_ (0) = param.as_double ();
                result.reason    = "Successful!";
            }
        }

        if (param.get_name () == "rt_surge_ki") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                surge_param_ (1) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_surge_kd") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                surge_param_ (2) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_surge_feedforward") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                surge_param_ (3) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_surge_satUpper") {
            double surgeSaturationLower = surge_param_ (5);
            if (param.as_double () < surgeSaturationLower) {
                result.reason =
                "Parameter has to be greater than the lower saturation limit";
                result.successful = false;
            } else {
                surge_param_ (4) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_surge_satLower") {
            double surgeSaturationUpper = surge_param_ (4);
            if (param.as_double () > surgeSaturationUpper) {
                result.reason =
                "Parameter has to be less than the upper saturation limit";
                result.successful = false;
            } else {
                surge_param_ (5) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_kp") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                sway_param_ (0) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_ki") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                sway_param_ (1) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_kd") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                sway_param_ (2) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_feedforward") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                sway_param_ (3) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_satUpper") {
            double swaySaturationLower = sway_param_ (5);
            if (param.as_double () < swaySaturationLower) {
                result.reason =
                "Parameter has to be greater than the lower saturation limit";
                result.successful = false;
            } else {
                sway_param_ (4) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_sway_satLower") {
            double swaySaturationUpper = sway_param_ (4);
            if (param.as_double () > swaySaturationUpper) {
                result.reason =
                "Parameter has to be less than the upper saturation limit";
                result.successful = false;
            } else {
                sway_param_ (5) = param.as_double ();
                result.reason   = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_kp") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                depth_param_ (0) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_ki") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                depth_param_ (1) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_kd") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                depth_param_ (2) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_feedforward") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                depth_param_ (3) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_satUpper") {
            double depthSaturationLower = depth_param_ (5);
            if (param.as_double () < depthSaturationLower) {
                result.reason =
                "Parameter has to be greater than the lower saturation limit";
                result.successful = false;
            } else {
                depth_param_ (4) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_depth_satLower") {
            double depthSaturationLower = depth_param_ (4);
            if (param.as_double () > depthSaturationLower) {
                result.reason =
                "Parameter has to be less than the upper saturation limit";
                result.successful = false;
            } else {
                depth_param_ (5) = param.as_double ();
                result.reason    = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_kp") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                yaw_param_ (0) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_ki") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                yaw_param_ (1) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_kd") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                yaw_param_ (2) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_feedforward") {
            if (param.as_double () < 0.0) {
                result.reason     = "Parameter has to be a positive double";
                result.successful = false;
            } else {
                yaw_param_ (3) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_satUpper") {
            double yawSaturationLower = yaw_param_ (5);
            if (param.as_double () < yawSaturationLower) {
                result.reason =
                "Parameter has to be greater than the lower saturation limit";
                result.successful = false;
            } else {
                yaw_param_ (4) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
        if (param.get_name () == "rt_yaw_satLower") {
            double yawSaturationUpper = yaw_param_ (4);
            if (param.as_double () > yawSaturationUpper) {
                result.reason =
                "Parameter has to be less than the upper saturation limit";
                result.successful = false;
            } else {
                yaw_param_ (5) = param.as_double ();
                result.reason  = "Successful!";
            }
        }
    }

    // Update controller parameters
    // Order: kp, ki, kd, feedforward, saturation upper, saturation lower
    surgeController_.setParameters (surge_param_ (0), surge_param_ (1), surge_param_ (2),
                                    surge_param_ (4), surge_param_ (5), false);
    swayController_.setParameters (sway_param_ (0), sway_param_ (1), sway_param_ (2),
                                   sway_param_ (4), sway_param_ (5), false);
    depthController_.setParameters (depth_param_ (0), depth_param_ (1), depth_param_ (2),
                                    depth_param_ (4), depth_param_ (5), false);
    yawController_.setParameters (yaw_param_ (0), yaw_param_ (1), yaw_param_ (2),
                                  yaw_param_ (4), yaw_param_ (5), true);


    return result;
};
