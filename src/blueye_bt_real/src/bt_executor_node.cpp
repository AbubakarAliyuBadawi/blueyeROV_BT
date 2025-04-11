#include "blueye_bt_real/bt_executor_node.hpp"
#include "blueye_bt_real/actions/navigate_waypoint_action.hpp"
#include "blueye_bt_real/conditions/battery_condition.hpp"

#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

using namespace std::chrono_literals;

namespace blueye_bt_real {

BTExecutorNode::BTExecutorNode()
    : rclcpp::Node("blueye_bt_executor")
{
    // Declare parameters
    this->declare_parameter("bt_filename", "main_mission.xml");
    this->declare_parameter("tick_rate_ms", 100);
    this->declare_parameter("drone_ip", "192.168.1.101");
    this->declare_parameter("drone_timeout", 30);
    
    // Get parameters
    std::string bt_filename = this->get_parameter("bt_filename").as_string();
    int tick_rate_ms = this->get_parameter("tick_rate_ms").as_int();
    
    // Construct BT file path
    std::string package_path = ament_index_cpp::get_package_share_directory("blueye_bt_real");
    bt_file_path_ = package_path + "/behavior_trees/" + bt_filename;
    
    RCLCPP_INFO(this->get_logger(), "BT file path: %s", bt_file_path_.c_str());
    
    // Connect to the drone first
    if (!connectToDrone()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to Blueye drone. Exiting.");
        return;
    }
    
    // Initialize factory with custom nodes
    initializeFactory();
    
    // Create and start timer for BT execution
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(tick_rate_ms),
        std::bind(&BTExecutorNode::timerCallback, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Blueye BT executor initialized");
}

BTExecutorNode::~BTExecutorNode()
{
    // Stop BT execution
    if (timer_) {
        timer_->cancel();
    }
    
    // Clear tree
    if (tree_) {
        tree_->haltTree();
    }
    
    RCLCPP_INFO(this->get_logger(), "Shutting down Blueye BT executor");
}

void BTExecutorNode::initializeFactory()
{
    // Register your custom BT nodes here with drone reference
    factory_.registerBuilder<NavigateWaypointAction>(
        "NavigateWaypoint",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<NavigateWaypointAction>(name, config, *drone_);
        });
    
    factory_.registerBuilder<BatteryCondition>(
        "BatteryCondition",
        [this](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<BatteryCondition>(name, config, *drone_);
        });
    
    RCLCPP_INFO(this->get_logger(), "BT factory initialized with custom nodes");
}

bool BTExecutorNode::connectToDrone()
{
    // Get drone connection parameters
    std::string drone_ip = this->get_parameter("drone_ip").as_string();
    int drone_timeout = this->get_parameter("drone_timeout").as_int();
    
    try {
        // Connect to the real Blueye drone using SDK
        RCLCPP_INFO(this->get_logger(), "Attempting to connect to Blueye drone at %s", drone_ip.c_str());
        
        drone_ = std::make_unique<blueye::sdk::Drone>(
            drone_ip,
            true,  // auto_connect
            drone_timeout
        );
        
        if (!drone_->connected) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to drone");
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to drone: %s", drone_->serial_number.c_str());
        RCLCPP_INFO(this->get_logger(), "Drone software version: %s", drone_->software_version.c_str());
        
        if (!drone_->in_control) {
            RCLCPP_INFO(this->get_logger(), "Taking control of drone...");
            drone_->take_control();
            RCLCPP_INFO(this->get_logger(), "Control of drone acquired");
        }
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to connect to drone: %s", e.what());
        return false;
    }
}

void BTExecutorNode::timerCallback()
{
    if (!tree_) {
        // First time initialization
        try {
            tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromFile(bt_file_path_));
            
            // Attach a logger
            auto logger = std::make_unique<BT::StdCoutLogger>(*tree_);
            
            RCLCPP_INFO(this->get_logger(), "Behavior tree created successfully");
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create behavior tree: %s", e.what());
            timer_->cancel();
            return;
        }
    }
    
    // Execute one tick of the behavior tree
    BT::NodeStatus status = tree_->tickOnce();
    
    // Check if the tree has completed
    if (status == BT::NodeStatus::SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "Behavior tree completed successfully");
        timer_->cancel();
    }
    else if (status == BT::NodeStatus::FAILURE) {
        RCLCPP_ERROR(this->get_logger(), "Behavior tree failed");
        timer_->cancel();
    }
}

void BTExecutorNode::run()
{
    // This method is currently unused, but could be used to manually control execution
    RCLCPP_INFO(this->get_logger(), "Starting BT execution");
    
    // Run the BT until completion or failure
    if (tree_) {
        while (rclcpp::ok()) {
            BT::NodeStatus status = tree_->tickOnce();
            if (status != BT::NodeStatus::RUNNING) {
                RCLCPP_INFO(this->get_logger(), "BT execution finished with status: %s",
                          status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
                break;
            }
            std::this_thread::sleep_for(100ms);
        }
    }
}

}  // namespace blueye_bt_real