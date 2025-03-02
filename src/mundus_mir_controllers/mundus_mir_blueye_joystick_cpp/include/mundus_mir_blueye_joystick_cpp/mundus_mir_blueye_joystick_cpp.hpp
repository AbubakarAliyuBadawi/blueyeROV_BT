#ifndef MUNDUS_MIR_BLUEYE_JOYSTICK_CPP_HPP
#define MUNDUS_MIR_BLUEYE_JOYSTICK_CPP_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using namespace std;

class JoystickController : public rclcpp::Node  {

    public:
        JoystickController();
        ~JoystickController();

    // Methods
    private:
        void joystick_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        void fetch_ros_parameters();
        void handle_buttons(const vector<int32_t>& buttons);

    // Variables
    private:

        // ROS 
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr cmd_pub_;
        rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

        // Configuration file parameters
        bool quadratic_mapping_, velocity_control_;
        double max_thrust_surge_, max_thrust_sway_, max_thrust_heave_, max_thrust_yaw_;
        double max_velocity_x_, max_velocity_y_, max_velocity_z_, max_angular_velocity_yaw_;
        string joy_topic_, cmd_topic_, vel_topic_;

        // Controller indexes, may vary from joystick to joystick..
        int right_joystick_x_axis_;
        int right_joystick_y_axis_;
        int left_joystick_x_axis_;
        int left_joystick_y_axis_;
        int x_button_;
        int circle_button_;
        int square_button_;
        int triangle_button_;

        // Control States
        bool joystick_active_ = true;
        bool ctrl_state_1_ = false;
        bool ctrl_state_2_ = false;
        bool ctrl_state_3_ = false;

        // Button states
        bool x_is_pressed_ = false;
        bool triangle_is_pressed_ = false;
        bool circle_is_pressed_ = false;
        bool square_is_pressed_ = false;

};
#endif