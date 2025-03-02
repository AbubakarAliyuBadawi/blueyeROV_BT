#include "mundus_mir_blueye_joystick_cpp/mundus_mir_blueye_joystick_cpp.hpp"

JoystickController::JoystickController() : Node("mundus_mir_blueye_joystick_cpp") {
    // Fetch parameters
    fetch_ros_parameters();

    // Initialize publisher and subscriber

    if (!velocity_control_) {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(cmd_topic_, 10);
        RCLCPP_INFO(get_logger(), "Publishing to %s", cmd_topic_.c_str());
    }
    else {
        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(vel_topic_, 10);
        RCLCPP_INFO(get_logger(), "Publishing to %s", vel_topic_.c_str());
    }
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(joy_topic_, 10, std::bind(&JoystickController::joystick_callback, this, std::placeholders::_1));

}

JoystickController::~JoystickController() {
    // Do nothing
}

void JoystickController::fetch_ros_parameters() {
    // Fetch parameters
    this->declare_parameter("quadratic_mapping", true);
    this->declare_parameter("max_thrust_surge", 500.0);
    this->declare_parameter("max_thrust_sway", 500.0);
    this->declare_parameter("max_thrust_heave", 500.0);
    this->declare_parameter("max_thrust_yaw", 500.0);
    this->declare_parameter("joy_topic", "/joy");
    this->declare_parameter("cmd_topic", "/blueye/force_cmd");

    this->declare_parameter("right_joystick_x_axis", 2);
    this->declare_parameter("right_joystick_y_axis", 3);
    this->declare_parameter("left_joystick_x_axis", 0);
    this->declare_parameter("left_joystick_y_axis", 1);
    this->declare_parameter("x_button", 0);
    this->declare_parameter("circle_button", 1);
    this->declare_parameter("square_button", 2);
    this->declare_parameter("triangle_button", 3);

    this->declare_parameter("velocity_control", false);
    this->declare_parameter("max_velocity_x", 0.5);
    this->declare_parameter("max_velocity_y", 0.5);
    this->declare_parameter("max_velocity_z", 0.5);
    this->declare_parameter("max_angular_velocity_yaw", 1.0); // Radians/s
    this->declare_parameter("velocity_topic", "/blueye/velocity_ref");

    this->get_parameter("quadratic_mapping", quadratic_mapping_);
    this->get_parameter("max_thrust_surge", max_thrust_surge_);
    this->get_parameter("max_thrust_sway", max_thrust_sway_);
    this->get_parameter("max_thrust_heave", max_thrust_heave_);
    this->get_parameter("max_thrust_yaw", max_thrust_yaw_);
    this->get_parameter("joy_topic", joy_topic_);
    this->get_parameter("cmd_topic", cmd_topic_);

    this->get_parameter("right_joystick_x_axis", right_joystick_x_axis_);
    this->get_parameter("right_joystick_y_axis", right_joystick_y_axis_);
    this->get_parameter("left_joystick_x_axis", left_joystick_x_axis_);
    this->get_parameter("left_joystick_y_axis", left_joystick_y_axis_);
    this->get_parameter("x_button", x_button_);
    this->get_parameter("circle_button", circle_button_);
    this->get_parameter("square_button", square_button_);
    this->get_parameter("triangle_button", triangle_button_);

    this->get_parameter("velocity_control", velocity_control_);
    this->get_parameter("max_velocity_x", max_velocity_x_);
    this->get_parameter("max_velocity_y", max_velocity_y_);
    this->get_parameter("max_velocity_z", max_velocity_z_);
    this->get_parameter("max_angular_velocity_yaw", max_angular_velocity_yaw_);
    this->get_parameter("velocity_topic", vel_topic_);
}

void JoystickController::joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

    // Check buttons to see if we should go to another control state
    handle_buttons(msg->buttons);

    // Do action based on current state
    if (joystick_active_) {

        double desired_surge, desired_sway, desired_heave, desired_yaw;

        if (!velocity_control_) {

        // Calculate desired thrust values
        if (quadratic_mapping_) {
            desired_surge = copysign(1.0, msg->axes[right_joystick_y_axis_]) * pow(msg->axes[right_joystick_y_axis_], 2) * max_thrust_surge_;
            desired_sway = copysign(1.0, msg->axes[right_joystick_x_axis_]) * pow(msg->axes[right_joystick_x_axis_], 2) * max_thrust_sway_;
            desired_heave = copysign(1.0, msg->axes[left_joystick_y_axis_]) * pow(msg->axes[left_joystick_y_axis_], 2) * max_thrust_heave_;
            desired_yaw = copysign(1.0, msg->axes[left_joystick_x_axis_]) * pow(msg->axes[left_joystick_x_axis_], 2) * max_thrust_yaw_;
        }
        else {
            desired_surge = msg->axes[right_joystick_y_axis_] * max_thrust_surge_;
            desired_sway = msg->axes[right_joystick_x_axis_] * max_thrust_sway_;
            desired_heave = msg->axes[left_joystick_y_axis_] * max_thrust_heave_;
            desired_yaw = msg->axes[left_joystick_x_axis_] * max_thrust_yaw_;
        }

        // Prepare a twist message
        auto wrench_msg = geometry_msgs::msg::WrenchStamped();
        auto header = std_msgs::msg::Header();
        header.frame_id = "body_frame";
        header.stamp = now();
        auto wrench = geometry_msgs::msg::Wrench();
        wrench.force.x = desired_surge;
        wrench.force.y = desired_sway;
        wrench.force.z = desired_heave;
        wrench.torque.x = 0.0;
        wrench.torque.y = 0.0;
        wrench.torque.z = desired_yaw;
        wrench_msg.header = header;
        wrench_msg.wrench = wrench;

        // Publish message
        cmd_pub_->publish(wrench_msg);
        }

        // Velocity Control
        else if (velocity_control_) {

        // Calculate desired thrust values
        if (quadratic_mapping_) {
            desired_surge = copysign(1.0, msg->axes[right_joystick_y_axis_]) * pow(msg->axes[right_joystick_y_axis_], 2) * max_velocity_x_;
            desired_sway = -copysign(1.0, msg->axes[right_joystick_x_axis_]) * pow(msg->axes[right_joystick_x_axis_], 2) * max_velocity_y_;
            desired_heave = copysign(1.0, msg->axes[left_joystick_y_axis_]) * pow(msg->axes[left_joystick_y_axis_], 2) * max_velocity_z_;
            desired_yaw = -copysign(1.0, msg->axes[left_joystick_x_axis_]) * pow(msg->axes[left_joystick_x_axis_], 2) * max_angular_velocity_yaw_;
        }
        else {
            desired_surge = msg->axes[right_joystick_y_axis_] * max_velocity_x_;
            desired_sway = -msg->axes[right_joystick_x_axis_] * max_velocity_y_;
            desired_heave = msg->axes[left_joystick_y_axis_] * max_velocity_z_;
            desired_yaw = -msg->axes[left_joystick_x_axis_] * max_angular_velocity_yaw_;
        }

        // Prepare a twist message
        auto twist_msg = geometry_msgs::msg::TwistStamped();
        auto header = std_msgs::msg::Header();
        header.frame_id = "body_frame";
        header.stamp = now();
        twist_msg.header = header;

        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = desired_surge;
        twist.linear.y = desired_sway;
        twist.linear.z = desired_heave;
        twist.angular.z = desired_yaw;
        twist_msg.twist = twist;

        // Publish message
        vel_pub_->publish(twist_msg);

        }

    }
    else if (ctrl_state_1_) {
        // Do a service call or something
    }
    // If we are in control state 2
    else if (ctrl_state_2_) {
        // Do a service call or something
    }
    // If we are in control state 3
    else if (ctrl_state_3_) {
        // Do a service call or something
    }
    

}

void JoystickController::handle_buttons(const vector<int32_t>& buttons) {

    // Go through all buttons and check if they are pressed
    if (buttons[x_button_] == 1 && !x_is_pressed_) {
        x_is_pressed_ = true;
        joystick_active_ = !joystick_active_;
        if (joystick_active_) {
            RCLCPP_INFO(get_logger(), "Joystick is now active");
        }
        else {
            RCLCPP_INFO(get_logger(), "Joystick is now inactive");
        }
    }
    else if (!buttons[x_button_] && x_is_pressed_) {
        x_is_pressed_ = false;
    }

    if (buttons[triangle_button_] == 1 && !triangle_is_pressed_) {
        triangle_is_pressed_ = true;
        ctrl_state_1_ = !ctrl_state_1_;
        if (ctrl_state_1_) {
            RCLCPP_INFO(get_logger(), "Control state 1 is now active");
        }
        else {
            RCLCPP_INFO(get_logger(), "Control state 1 is now inactive");
        }
    }
    else if (!buttons[triangle_button_] && triangle_is_pressed_) {
        triangle_is_pressed_ = false;
    }

    if (buttons[circle_button_] == 1 && !circle_is_pressed_) {
        circle_is_pressed_ = true;
        ctrl_state_2_ = !ctrl_state_2_;
        if (ctrl_state_2_) {
            RCLCPP_INFO(get_logger(), "Control state 2 is now active");
        }
        else {
            RCLCPP_INFO(get_logger(), "Control state 2 is now inactive");
        }
    }
    else if (!buttons[circle_button_] && circle_is_pressed_) {
        circle_is_pressed_ = false;
    }

    if (buttons[square_button_] == 1 && !square_is_pressed_) {
        square_is_pressed_ = true;
        ctrl_state_3_ = !ctrl_state_3_;
        if (ctrl_state_3_) {
            RCLCPP_INFO(get_logger(), "Control state 3 is now active");
        }
        else {
            RCLCPP_INFO(get_logger(), "Control state 3 is now inactive");
        }
    }
    else if (!buttons[square_button_] && square_is_pressed_) {
        square_is_pressed_ = false;
    }
}