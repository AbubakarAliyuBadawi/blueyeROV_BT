#pragma once
#include <behaviortree_cpp/bt_factory.h>

namespace rov_mission_bt {

class MissionFactory {
public:
    MissionFactory();
    
    // Register all behavior types
    void registerBehaviors();
    
    // Get factory instance
    BT::BehaviorTreeFactory& getFactory() { return factory_; }

private:
    void registerWaypointBehaviors();
    void registerManipulationBehaviors();
    void registerSensorBehaviors();
    
    BT::BehaviorTreeFactory factory_;
};

} // namespace rov_mission_bt