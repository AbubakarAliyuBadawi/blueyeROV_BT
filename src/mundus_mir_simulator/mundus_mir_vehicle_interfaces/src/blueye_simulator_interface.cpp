#include "mundus_mir_vehicle_interfaces/blueye_simulator_interface.hpp"

// 

BlueyeThrustAllocator::BlueyeThrustAllocator() : Node("mundus_mir_thruster_allocation") {
    // Fetch parameters
    fetch_ros_parameters();

    // Initialize publisher and subscriber
    t1_pub_ = this->create_publisher<std_msgs::msg::Float64>(t1_topic_, 1);
    t2_pub_ = this->create_publisher<std_msgs::msg::Float64>(t2_topic_, 1);
    t3_pub_ = this->create_publisher<std_msgs::msg::Float64>(t3_topic_, 1);
    t4_pub_ = this->create_publisher<std_msgs::msg::Float64>(t4_topic_, 1);
    desired_force_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(cmd_force_topic_, 10, std::bind(&BlueyeThrustAllocator::desired_force_callback, this, std::placeholders::_1));

}

BlueyeThrustAllocator::~BlueyeThrustAllocator() {

}

void BlueyeThrustAllocator::fetch_ros_parameters() {
    // Fetch parameters
    this->declare_parameter("cmd_force_topic", "/blueye/cmd_force");
    this->declare_parameter("t1_topic", "/blueye/thruster_1/cmd_vel");
    this->declare_parameter("t2_topic", "/blueye/thruster_2/cmd_vel");
    this->declare_parameter("t3_topic", "/blueye/thruster_3/cmd_vel");
    this->declare_parameter("t4_topic", "/blueye/thruster_4/cmd_vel");

    this->get_parameter("cmd_force_topic", cmd_force_topic_);
    this->get_parameter("t1_topic", t1_topic_);
    this->get_parameter("t2_topic", t2_topic_);
    this->get_parameter("t3_topic", t3_topic_);
    this->get_parameter("t4_topic", t4_topic_);
}

void BlueyeThrustAllocator::desired_force_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {

    // Calculate individual thrusts
    Eigen::VectorXd desired_thrusts(4);
    desired_thrusts(0) = msg->wrench.force.x;
    desired_thrusts(1) = msg->wrench.force.y;
    desired_thrusts(2) = msg->wrench.force.z;
    desired_thrusts(3) = msg->wrench.torque.z;

    Eigen::MatrixXd TAM(4,4);
    TAM << 0.5, 0.5, 0.0, 0.0,
           0.0, 0.0, -1.0, 0.0,
           0.0, 0.0, 0.0, 1.0,
           0.5, -0.5, 0.0, 0.0;

    Eigen::MatrixXd TAM_pseudo_inverse = TAM.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::VectorXd thrust_commands = TAM_pseudo_inverse * desired_thrusts;

    // Publish thrust commands
    std_msgs::msg::Float64 t1_msg, t2_msg, t3_msg, t4_msg;
    t1_msg.data = thrust_commands(0);
    t2_msg.data = thrust_commands(1);
    t3_msg.data = thrust_commands(2);
    t4_msg.data = thrust_commands(3);
    t1_pub_->publish(t1_msg);
    t2_pub_->publish(t2_msg);
    t3_pub_->publish(t3_msg);
    t4_pub_->publish(t4_msg);
}

// Starts and spins blueye vehicle interface
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlueyeThrustAllocator>());
    rclcpp::shutdown();
    return -1;
}