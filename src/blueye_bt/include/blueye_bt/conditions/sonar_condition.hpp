#ifndef SONAR_CONDITION_HPP
#define SONAR_CONDITION_HPP

#include <behaviortree_cpp/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>

class CheckSonarStatus : public BT::ConditionNode {
private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_sub_;
    bool sonar_ok_ = false;
    rclcpp::Time last_msg_time_;
    double timeout_seconds_ = 1.0; // Increased timeout for development

public:
    CheckSonarStatus(const std::string& name, const BT::NodeConfiguration& config);
    
    // Port definition
    static BT::PortsList providedPorts() {
        return BT::PortsList({
            BT::InputPort<double>("timeout_seconds", 1.0, "Maximum time without sonar messages before reporting failure")
        });
    }
    
    BT::NodeStatus tick() override;
    
private:
    void sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg);
};

#endif // SONAR_CONDITION_HPP