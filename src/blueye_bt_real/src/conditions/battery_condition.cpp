#include "blueye_bt_real/conditions/battery_condition.hpp"

namespace blueye_bt_real {

BatteryCondition::BatteryCondition(const std::string& name, const BT::NodeConfiguration& config, blueye::sdk::Drone& drone)
    : BT::ConditionNode(name, config),
      drone_(drone),
      threshold_(20.0),  // Default 20% threshold
      logger_(rclcpp::get_logger("battery_condition"))
{
}

BT::PortsList BatteryCondition::providedPorts()
{
    BT::PortsList ports;
    ports.insert(BT::InputPort<double>("threshold", "Battery threshold percentage"));
    return ports;
}

BT::NodeStatus BatteryCondition::tick()
{
    // Get threshold from port
    if (!getInput("threshold", threshold_)) {
        threshold_ = 20.0;  // Default to 20% if not specified
    }
    
    try {
        // Get real battery level from drone
        double battery_level = drone_.battery.level;
        
        RCLCPP_INFO(logger_, "Battery level: %.1f%%, Threshold: %.1f%%", 
                   battery_level, threshold_);
        
        // Return SUCCESS if battery is above threshold (meaning continue with mission)
        // Return FAILURE if battery is below threshold (will trigger fallback to docking)
        return battery_level > threshold_ ? 
               BT::NodeStatus::SUCCESS : 
               BT::NodeStatus::FAILURE;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Error checking battery: %s", e.what());
        return BT::NodeStatus::FAILURE;  // Safe default is to dock if we can't check battery
    }
}

}  // namespace blueye_bt_real