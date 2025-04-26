// learning_mission_selector.hpp
#ifndef LEARNING_MISSION_SELECTOR_HPP
#define LEARNING_MISSION_SELECTOR_HPP

#include <behaviortree_cpp/control_node.h>
#include <marine_acoustic_msgs/msg/projected_sonar_image.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <fstream>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

extern rclcpp::Node::SharedPtr g_node;

class LearningMissionSelector : public BT::ControlNode {
public:
    LearningMissionSelector(const std::string& name, const BT::NodeConfiguration& config);
    
    static BT::PortsList providedPorts();
    
    BT::NodeStatus tick() override;

private:
    struct MissionState {
        int battery_level;      // 0: Low, 1: Medium, 2: High
        bool camera_working;    // true if camera is operational
        bool sonar_working;     // true if sonar is operational
        double distance_to_dock; // discretized: 0: Near, 1: Medium, 2: Far
        
        // For Q-table lookup, we need states to be comparable
        bool operator<(const MissionState& other) const {
            return std::tie(battery_level, camera_working, sonar_working, distance_to_dock) < 
                   std::tie(other.battery_level, other.camera_working, other.sonar_working, other.distance_to_dock);
        }
        
        std::string toString() const {
            std::stringstream ss;
            ss << "Battery: " << (battery_level == 0 ? "Low" : battery_level == 1 ? "Medium" : "High")
               << ", Camera: " << (camera_working ? "Working" : "Offline")
               << ", Sonar: " << (sonar_working ? "Working" : "Offline")
               << ", Distance to Dock: " << (distance_to_dock == 0 ? "Near" : distance_to_dock == 1 ? "Medium" : "Far");
            return ss.str();
        }
    };
    
    using MissionOrder = std::vector<size_t>;
    
    // Q-table and state tracking
    std::map<MissionState, std::map<MissionOrder, double>> q_table_;
    MissionState current_state_;
    MissionOrder current_order_;
    std::vector<size_t> children_indices_;

    // State tracking for execution
    bool currently_executing_ = false;
    size_t current_child_index_ = 0;
    
    // Learning parameters
    double learning_rate_;
    double discount_factor_;
    double exploration_rate_;
    
    // Random number generator
    std::mt19937 rng_{std::random_device{}()};
    
    bool learning_enabled_ = true;
    std::string explanation_;

    // Sensor state tracking
    double current_battery_percentage_ = 100.0;  // Default to 80%
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr battery_sub_;
    
    // Camera status tracking
    bool camera_working_ = true;  // Default to working
    rclcpp::Time last_camera_msg_time_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    double camera_timeout_seconds_ = 10.0;

    // Sonar status tracking
    bool sonar_working_ = true;  // Default to working
    rclcpp::Time last_sonar_msg_time_;
    rclcpp::Subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>::SharedPtr sonar_sub_;
    double sonar_timeout_seconds_ = 10.0;

    // Distance to dock tracking
    double current_distance_to_dock_ = 0.0;  // Default to 50m
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;

    // Callback for battery percentage
    void batteryCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_battery_percentage_ = msg->data;
        RCLCPP_DEBUG(g_node->get_logger(), "Battery percentage updated: %.1f%%", current_battery_percentage_);
    }

    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        // Update last message time
        last_camera_msg_time_ = g_node->now();
        
        // Check if image data is valid
        camera_working_ = (msg->width > 0 && msg->height > 0 && !msg->data.empty());
        
        RCLCPP_DEBUG(g_node->get_logger(), "Camera status updated: %s", 
                   camera_working_ ? "Working" : "Not working");
    }

    // Callback for sonar
    void sonarCallback(const marine_acoustic_msgs::msg::ProjectedSonarImage::SharedPtr msg) {
        // Update last message time
        last_sonar_msg_time_ = g_node->now();
        
        // Consider the sonar working as long as we're receiving messages
        sonar_working_ = true;
        
        RCLCPP_DEBUG(g_node->get_logger(), "Sonar data received, status: Working");
    }

    // Callback for distance to dock
    void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_distance_to_dock_ = msg->data;
        RCLCPP_DEBUG(g_node->get_logger(), "Distance to dock updated: %.2f m", current_distance_to_dock_);
    }

    // Helper methods
    std::string missionOrderToString(const MissionOrder& order);
    MissionState getCurrentState();
    MissionOrder getRandomOrder();
    MissionOrder getBestOrder(const MissionState& state);
    std::vector<std::pair<MissionOrder, double>> getTopOrders(const MissionState& state, int n);
    double getQValue(const MissionState& state, const MissionOrder& order);
    void updateQValue(const MissionState& state, const MissionOrder& order, double reward);
    double calculateReward(const MissionState& state, size_t completed_task, bool success);
    void saveQTable(const std::string& filename);
    void loadQTable(const std::string& filename);
};

#endif // LEARNING_MISSION_SELECTOR_HPP