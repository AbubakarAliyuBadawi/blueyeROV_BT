#include "rov_mission_bt/mission_control/mission_loader.hpp"
#include "rov_mission_bt/mission_control/mission_factory.hpp"
#include "rov_mission_bt/utils/node_configuration.hpp"
#include "rov_mission_bt/behaviors/waypoint_behaviors.hpp"
#include "rov_mission_bt/conditions/battery_condition.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/controls/sequence_node.h>
#include <behaviortree_cpp_v3/controls/fallback_node.h>
#include <behaviortree_cpp_v3/controls/parallel_node.h>
#include <behaviortree_cpp_v3/decorators/inverter_node.h>
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
    // Setup signal handling
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("rov_mission");  // Match the name in launch file

    // Declare the parameter with a default value
    g_node->declare_parameter("behavior_tree_path", "");

    BT::BehaviorTreeFactory factory;

    // Register Control Nodes
    factory.registerNodeType<BT::RetryNode>("RetryNode");

    // Register your custom action nodes
    factory.registerNodeType<ClearWaypoints>("ClearWaypoints");
    factory.registerNodeType<SetWaypoint>("SetWaypoint");
    factory.registerNodeType<ExecuteWaypoint>("ExecuteWaypoint");
    factory.registerNodeType<CheckBatteryLevel>("CheckBatteryLevel");  
    factory.registerNodeType<StationKeeping>("StationKeeping");

    try {
        // Get the behavior tree path from parameter
        std::string mission_file;
        if (!g_node->get_parameter("behavior_tree_path", mission_file)) {
            throw std::runtime_error("Failed to get behavior_tree_path parameter");
        }
        
        // Log the path we're trying to load
        RCLCPP_INFO(g_node->get_logger(), "Loading behavior tree from: %s", mission_file.c_str());

        // Verify file exists before trying to load it
        if (access(mission_file.c_str(), F_OK) == -1) {
            throw std::runtime_error("Behavior tree file does not exist: " + mission_file);
        }

        // Create behavior tree from XML file
        auto tree = factory.createTreeFromFile(mission_file);
        RCLCPP_INFO(g_node->get_logger(), "Behavior tree created successfully");

        // Add the publisher AFTER tree creation
        BT::PublisherZMQ publisher_zmq(tree);
        RCLCPP_INFO(g_node->get_logger(), "ZMQ publisher created. You can monitor the tree using Groot");

        // Main loop
        while (rclcpp::ok() && g_program_running) {
            auto status = tree.tickRoot();
            rclcpp::spin_some(g_node);
            
            if (status != BT::NodeStatus::RUNNING) {
                RCLCPP_INFO(g_node->get_logger(), "Tree finished with status: %s", 
                           status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                break;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Clean shutdown sequence
        RCLCPP_INFO(g_node->get_logger(), "Starting clean shutdown...");
        
        // First halt the tree
        tree.haltTree();
        RCLCPP_INFO(g_node->get_logger(), "Tree halted successfully");
        
        // Give some time for final cleanup
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        // Final cleanup
        RCLCPP_INFO(g_node->get_logger(), "Shutdown complete");

    } catch (const std::exception& e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), "Exception caught: %s", e.what());
        }
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    // Clean up
    g_node.reset();
    rclcpp::shutdown();
    return 0;
}