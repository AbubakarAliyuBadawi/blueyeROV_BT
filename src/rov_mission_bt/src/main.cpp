#include "rov_mission_bt/mission_control/mission_loader.hpp"
#include "rov_mission_bt/mission_control/mission_factory.hpp"
#include "rov_mission_bt/utils/node_configuration.hpp"
#include "rov_mission_bt/behaviors/waypoint_behaviors.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>

// Global node for service clients
rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_program_running{true};

// Signal handler function
void signalHandler(int signum) {
    RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received.", signum);
    g_program_running = false;
}

int main(int argc, char **argv) {
    // Setup signal handling
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("rov_mission_bt");

    BT::BehaviorTreeFactory factory;
    
    // Register the custom nodes
    factory.registerNodeType<ClearWaypoints>("ClearWaypoints");
    factory.registerNodeType<SetWaypoint>("SetWaypoint");
    factory.registerNodeType<ExecuteWaypoint>("ExecuteWaypoint");

    try {
        // Create behavior tree from XML file
        std::string mission_file = "behavior_tree/dock_undock_mission.xml";
        auto tree = factory.createTreeFromFile(mission_file);
        RCLCPP_INFO(g_node->get_logger(), "Behavior tree created successfully");

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
        
        // Clear node after last log message
        g_node.reset();

    } catch (const std::exception& e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), "Exception caught: %s", e.what());
        }
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}