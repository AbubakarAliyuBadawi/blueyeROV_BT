#include "blueye_bt/behaviors/navigate_to_waypoint.hpp"
#include "blueye_bt/behaviors/station_keeping.hpp"
#include "blueye_bt/conditions/battery_condition.hpp"
#include "blueye_bt/conditions/camera_condition.hpp"
#include "blueye_bt/conditions/sonar_condition.hpp"
#include "blueye_bt/conditions/system_watchdog.hpp"
#include "blueye_bt/behaviors/load_mission_requirements.hpp"
#include "blueye_bt/conditions/blackboard_condition.hpp"
#include "blueye_bt/actions/altitude_control_action.hpp"
// #include "blueye_bt/loggers/timeline_logger.hpp"
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <memory>
#include <chrono>
#include <thread>
#include <unistd.h>

// Global node for service clients
rclcpp::Node::SharedPtr g_node;
std::atomic<bool> g_program_running{true};

void signalHandler(int signum) {
    if (g_node) {
        RCLCPP_INFO(g_node->get_logger(), "Interrupt signal (%d) received.", signum);
    }
    g_program_running = false;
}

int main(int argc, char **argv) {
    signal(SIGINT, signalHandler);
    
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("blueye_bt");
    g_node->declare_parameter("behavior_tree_path", "");
    g_node->declare_parameter("target_altitude", 2.0); 


    BT::BehaviorTreeFactory factory;

    // Register basic BT nodes
    factory.registerNodeType<BT::RetryNode>("RetryNode");
    factory.registerNodeType<BT::SequenceNode>("SequenceNode");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");

    // Register LoadMissionRequirements
    factory.registerBuilder<LoadMissionRequirements>(
        "LoadMissionRequirements",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<LoadMissionRequirements>(name, config);
        });
    
    // Register primary navigation nodes
    factory.registerBuilder<NavigateToWaypoint>(
        "NavigateToWaypoint",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<NavigateToWaypoint>(name, config);
        });
    
    factory.registerBuilder<StationKeeping>(
        "StationKeeping",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<StationKeeping>(name, config);
        });

    // Register condition nodes
    factory.registerBuilder<CheckCameraStatus>(
        "CheckCameraStatus",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<CheckCameraStatus>(name, config);
        });

        // Register sonar condition
    factory.registerBuilder<CheckSonarStatus>(
        "CheckSonarStatus",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<CheckSonarStatus>(name, config);
        });

    // Register system watchdog
    factory.registerBuilder<SystemWatchdog>(
        "SystemWatchdog",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<SystemWatchdog>(name, config);
        });
    
    // Register condition nodes
    factory.registerBuilder<CheckBatteryLevel>(
        "CheckBatteryLevel",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<CheckBatteryLevel>(name, config);
        });

    // Register altitude control action
    factory.registerBuilder<AltitudeControlAction>(
        "AltitudeControlAction",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<AltitudeControlAction>(name, config);
        });

    try {
        std::string mission_file;
        if (!g_node->get_parameter("behavior_tree_path", mission_file)) {
            throw std::runtime_error("Failed to get behavior_tree_path parameter");
        }
        
        RCLCPP_INFO(g_node->get_logger(), "Loading behavior tree from: %s", mission_file.c_str());
        if (access(mission_file.c_str(), F_OK) == -1) {
            throw std::runtime_error("Behavior tree file does not exist: " + mission_file);
        }

        auto tree = factory.createTreeFromFile(mission_file);
        RCLCPP_INFO(g_node->get_logger(), "Behavior tree created successfully");

        BT::Groot2Publisher publisher(tree, 6677);
        RCLCPP_INFO(g_node->get_logger(), "Groot2 publisher created on port 6677. You can monitor the tree using Groot2");

        // auto timeline_logger = std::make_shared<blueye_bt::TimelineLogger>(tree, g_node);
        // RCLCPP_INFO(g_node->get_logger(), "Timeline logger added. Data will be saved to /tmp/bt_timeline_data.csv");

        const auto sleep_ms = std::chrono::milliseconds(100);
        auto status = BT::NodeStatus::RUNNING;

        while (rclcpp::ok() && g_program_running) {
            status = tree.tickWhileRunning(sleep_ms);
            rclcpp::spin_some(g_node);
            
            if (BT::isStatusCompleted(status)) {
                RCLCPP_INFO(g_node->get_logger(), "Tree finished with status: %s", 
                           status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
            }
        }

        RCLCPP_INFO(g_node->get_logger(), "Starting clean shutdown...");
        tree.haltTree();
        RCLCPP_INFO(g_node->get_logger(), "Tree halted successfully");
        
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        RCLCPP_INFO(g_node->get_logger(), "Shutdown complete");

    } catch (const std::exception& e) {
        if (g_node) {
            RCLCPP_ERROR(g_node->get_logger(), "Exception caught: %s", e.what());
        }
        g_node.reset();
        rclcpp::shutdown();
        return 1;
    }

    g_node.reset();
    rclcpp::shutdown();
    return 0;
}