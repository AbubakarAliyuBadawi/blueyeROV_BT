#ifndef BLUEYE_BT_REAL_BT_EXECUTOR_NODE_HPP
#define BLUEYE_BT_REAL_BT_EXECUTOR_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <string>
#include <memory>
#include <blueye/sdk/drone.hpp>

namespace blueye_bt_real {

class BTExecutorNode : public rclcpp::Node {
public:
    BTExecutorNode();
    ~BTExecutorNode();
    
    void run();

private:
    // BehaviorTree.CPP factory
    BT::BehaviorTreeFactory factory_;
    
    // Behavior tree
    std::unique_ptr<BT::Tree> tree_;
    
    // Blueye SDK drone
    std::unique_ptr<blueye::sdk::Drone> drone_;
    
    // ROS Timer for BT tick
    rclcpp::TimerBase::SharedPtr timer_;
    
    // BT file path
    std::string bt_file_path_;
    
    // Initialize behavior tree factory with custom nodes
    void initializeFactory();
    
    // Timer callback for BT ticking
    void timerCallback();
    
    // Connect to the Blueye drone
    bool connectToDrone();
};

}  // namespace blueye_bt_real

#endif  // BLUEYE_BT_REAL_BT_EXECUTOR_NODE_HPP