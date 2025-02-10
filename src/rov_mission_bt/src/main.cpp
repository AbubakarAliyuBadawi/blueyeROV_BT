#include "rov_mission_bt/mission_control/mission_loader.hpp"
#include "rov_mission_bt/mission_control/mission_factory.hpp"
#include "rov_mission_bt/utils/node_configuration.hpp"
#include "rov_mission_bt/behaviors/waypoint_behaviors.hpp"
#include "rov_mission_bt/conditions/battery_condition.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
// Change these includes

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
// Note: Control nodes are now included differently in v4
// #include "behaviortree_cpp/loggers/bt_zmq_publisher.h"
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/controls/sequence.h>
#include <behaviortree_cpp/controls/fallback.h>
#include <behaviortree_cpp/controls/parallel.h>
#include <behaviortree_cpp/decorators/inverter.h>
#include <memory>
#include <chrono>
#include <thread>

// Global node for service clients
rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_program_running{true};

// Signal handler function
void signalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received.", signum);
    }
    g_program_running = false;
}

int main(int argc, char **argv) {
    // Setup signal handling and ROS2 initialization stays the same
    
    BT::BehaviorTreeFactory factory;

    // Register nodes using v4 style
    // RetryNode registration
    factory.registerNodeType<BT::RetryNode>("RetryNode");

    // Register custom nodes using v4 builder pattern
    auto builder_clear = [](const std::string& name, const BT::NodeConfig& config)
    {
        return std::make_unique<ClearWaypoints>(name, config);
    };
    factory.registerBuilder<ClearWaypoints>("ClearWaypoints", builder_clear);

    auto builder_set = [](const std::string& name, const BT::NodeConfig& config)
    {
        return std::make_unique<SetWaypoint>(name, config);
    };
    factory.registerBuilder<SetWaypoint>("SetWaypoint", builder_set);

    auto builder_execute = [](const std::string& name, const BT::NodeConfig& config)
    {
        return std::make_unique<ExecuteWaypoint>(name, config);
    };
    factory.registerBuilder<ExecuteWaypoint>("ExecuteWaypoint", builder_execute);

    auto builder_battery = [](const std::string& name, const BT::NodeConfig& config)
    {
        return std::make_unique<CheckBatteryLevel>(name, config);
    };
    factory.registerBuilder<CheckBatteryLevel>("CheckBatteryLevel", builder_battery);

    auto builder_station = [](const std::string& name, const BT::NodeConfig& config)
    {
        return std::make_unique<StationKeeping>(name, config);
    };
    factory.registerBuilder<StationKeeping>("StationKeeping", builder_station);

    try {
        // Tree creation and main loop remain largely the same
        // but status checking might need updates
        
        auto tree = factory.createTreeFromFile(mission_file);
        RCLCPP_INFO(g_node->get_logger(), "Behavior tree created successfully");

        // ZMQ publisher creation remains the same
        BT::PublisherZMQ publisher_zmq(tree);
        RCLCPP_INFO(g_node->get_logger(), "ZMQ publisher created. You can monitor the tree using Groot");

        // Main loop remains the same but with potentially updated status handling
        while (rclcpp::ok() && g_program_running) {
            auto status = tree.tickOnce();  // Note: tickRoot() is now tickOnce() in v4
            rclcpp::spin_some(g_node);
            
            if (status != BT::NodeStatus::RUNNING) {
                RCLCPP_INFO(g_node->get_logger(), "Tree finished with status: %s", 
                           status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Cleanup remains the same
    }
    catch(const std::exception& e) {
        // Exception handling remains the same
    }

    return 0;
}
