#ifndef MUNDUS_MIR_THRUSTER_ALLOCATION_HPP_
#define MUNDUS_MIR_THRUSTER_ALLOCATION_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>


using namespace std;

class BlueyeThrustAllocator : public rclcpp::Node {

    public:
        BlueyeThrustAllocator();
        ~BlueyeThrustAllocator();
    
    private:
        void desired_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void fetch_ros_parameters();

    private:
            
            // ROS 
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr desired_force_sub_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t1_pub_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t2_pub_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t3_pub_;
            rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr t4_pub_;
    
            // Configuration file parameters
            string cmd_force_topic_, t1_topic_, t2_topic_, t3_topic_, t4_topic_;

};

#endif