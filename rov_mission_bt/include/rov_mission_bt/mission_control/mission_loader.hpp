#pragma once
#include <string>
#include <filesystem>
#include <behaviortree_cpp_v3/bt_factory.h>

namespace rov_mission_bt {

class MissionLoader {
public:
    MissionLoader(const std::string& missions_directory);
    
    // Load mission from file
    BT::Tree loadMission(const std::string& mission_name);
    
    // Get available missions
    std::vector<std::string> getAvailableMissions() const;
    
    // Validate mission XML
    bool validateMission(const std::string& mission_xml);

private:
    std::filesystem::path missions_dir_;
};

} // namespace rov_mission_bt