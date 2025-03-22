// camera_condition.hpp
#ifndef CAMERA_CONDITION_HPP
#define CAMERA_CONDITION_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CheckCameraStatus : public BT::ConditionNode {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  
  bool camera_ok_ = false;
  rclcpp::Time last_msg_time_;
  double timeout_seconds_ = 2.0;  // How long without messages before considering the camera faulty

public:
  CheckCameraStatus(const std::string& name, const BT::NodeConfiguration& config);
  
  // Port definition
  static BT::PortsList providedPorts() {
    return BT::PortsList({
      BT::InputPort<double>("timeout_seconds", 2.0, "Maximum time without camera messages before reporting failure")
    }); 
  }
  
  BT::NodeStatus tick() override;
  
private:
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif // CAMERA_CONDITION_HPP