// mission_manager.cpp (partial update for init_bt_factory function)
#include "blueye_bt_alt/managers/mission_manager.hpp"
#include "blueye_bt_alt/behaviors/navigate_to_waypoint.hpp"
#include "blueye_bt_alt/behaviors/station_keeping.hpp"
#include "blueye_bt_alt/conditions/battery_condition.hpp"

// Other functions remain unchanged

void MissionManager::init_bt_factory() {
    // Set the global g_node pointer to point to this node
    g_node = shared_from_this();
    
    // Register your existing node types here
    factory_.registerNodeType<BT::RetryNode>("RetryNode");
    factory_.registerNodeType<BT::SequenceNode>("SequenceNode");
    
    // Register NavigateToWaypoint
    factory_.registerBuilder<NavigateToWaypoint>(
        "NavigateToWaypoint",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<NavigateToWaypoint>(name, config);
        });

    // Register StationKeeping
    factory_.registerBuilder<StationKeeping>(
        "StationKeeping",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<StationKeeping>(name, config);
        });

    // Register CheckBatteryLevel
    factory_.registerBuilder<CheckBatteryLevel>(
        "CheckBatteryLevel",
        [](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<CheckBatteryLevel>(name, config);
        });
        
    RCLCPP_INFO(this->get_logger(), "Behavior Tree factory initialized");
}