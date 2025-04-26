#include "rclcpp/rclcpp.hpp"
#include "mundus_mir_msgs/msg/battery_status.hpp"
#include "std_msgs/msg/float64.hpp"

class BatteryPercentageNode : public rclcpp::Node
{
public:
  BatteryPercentageNode() : Node("battery_percentage_node")
  {
    // Create subscriber to the battery status topic
    battery_subscriber_ = this->create_subscription<mundus_mir_msgs::msg::BatteryStatus>(
      "/blueye/battery", 10, 
      std::bind(&BatteryPercentageNode::batteryCallback, this, std::placeholders::_1));
      
    // Create publisher for battery percentage
    percentage_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
      "/blueye/battery_percentage", 10);
      
    RCLCPP_INFO(this->get_logger(), "Battery percentage node initialized");
  }

private:
  void batteryCallback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg)
  {
    // Create percentage message
    auto percentage_msg = std::make_unique<std_msgs::msg::Float64>();
    percentage_msg->data = msg->state_of_charge * 100.0;
    
    // Publish the percentage
    percentage_publisher_->publish(*percentage_msg);
  }
  
  rclcpp::Subscription<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr battery_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr percentage_publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryPercentageNode>());
  rclcpp::shutdown();
  return 0;
}