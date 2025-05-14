#include "blueye_bt/control_nodes/learning_mission_selector.hpp"
#include <algorithm>
#include <numeric>
#include <limits>
#include <tuple>
#include <sstream>

LearningMissionSelector::LearningMissionSelector(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ControlNode(name, config) {
    
    // Default parameters
    learning_rate_ = 0.1;
    discount_factor_ = 0.9;
    exploration_rate_ = 0.3;

    // Debug info about expected children
    RCLCPP_INFO(g_node->get_logger(), "LearningMissionSelector '%s' initialized.", name.c_str());
    RCLCPP_INFO(g_node->get_logger(), "This node should have 3 children in order:");
    RCLCPP_INFO(g_node->get_logger(), "  0: Pipeline Inspection");
    RCLCPP_INFO(g_node->get_logger(), "  1: Wreckage Inspection");
    RCLCPP_INFO(g_node->get_logger(), "  2: Return to Dock");

    // Set up subscriptions for sensor data
    battery_sub_ = g_node->create_subscription<std_msgs::msg::Float64>(
        "/blueye/battery_percentage", 10,
        std::bind(&LearningMissionSelector::batteryCallback, this, std::placeholders::_1));
    
    // Initialize camera subscription
    camera_sub_ = g_node->create_subscription<sensor_msgs::msg::Image>(
        "/blueye/camera_1/image_raw", 10,
        std::bind(&LearningMissionSelector::cameraCallback, this, std::placeholders::_1));
    
    // Initialize sonar subscription
    sonar_sub_ = g_node->create_subscription<marine_acoustic_msgs::msg::ProjectedSonarImage>(
        "/mundus_mir/sonar", 10,
        std::bind(&LearningMissionSelector::sonarCallback, this, std::placeholders::_1));
    
    // Initialize distance subscription
    distance_sub_ = g_node->create_subscription<std_msgs::msg::Float64>(
        "/blueye/distance_to_dock", 10,
        std::bind(&LearningMissionSelector::distanceCallback, this, std::placeholders::_1));

    // Initialize last camera message time
    last_camera_msg_time_ = g_node->now();
    last_sonar_msg_time_ = g_node->now();

    // Try to load Q-table from file
    loadQTable("/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt/scripts/mission_q_table.txt");
    
    RCLCPP_INFO(g_node->get_logger(), "Learning Mission Selector initialized");
}

BT::PortsList LearningMissionSelector::providedPorts() {
    return {
        BT::InputPort<double>("learning_rate", 0.1, "Alpha parameter for Q-learning"),
        BT::InputPort<double>("discount_factor", 0.9, "Gamma parameter for Q-learning"),
        BT::InputPort<double>("exploration_rate", 0.3, "Epsilon for exploration strategy"),
        BT::OutputPort<std::string>("explanation", "Explanation of decision making")
    };
}

BT::NodeStatus LearningMissionSelector::tick() {
    // Print children info the first time for debugging
    static bool first_tick = true;
    if (first_tick) {
        RCLCPP_INFO(g_node->get_logger(), "Children nodes (%zu):", children_nodes_.size());
        for (size_t i = 0; i < children_nodes_.size(); ++i) {
            RCLCPP_INFO(g_node->get_logger(), "  Child %zu: %s", i, children_nodes_[i]->name().c_str());
        }
        first_tick = false;
    }

    // If we're already executing a mission sequence, continue from where we left off
    if (currently_executing_) {
        // Get the current child we're executing
        if (current_child_index_ >= current_order_.size()) {
            // We've completed all children, this is a failure case
            currently_executing_ = false;
            
            // Calculate reward for failure
            double reward = calculateReward(current_state_, current_order_.back(), false);
            
            // Update Q-table
            if (learning_enabled_) {
                updateQValue(current_state_, current_order_, reward);
            }
            
            return BT::NodeStatus::FAILURE;
        }
        
        // Safety check for valid indices
        size_t idx = current_order_[current_child_index_];
        if (idx >= children_nodes_.size()) {
            RCLCPP_ERROR(g_node->get_logger(), 
                        "Invalid child index: %zu (max: %zu). Using index 0 instead.", 
                        idx, children_nodes_.size()-1);
            idx = 0;  // Default to first child
        }
        
        BT::NodeStatus child_status = children_nodes_[idx]->executeTick();
        
        if (child_status == BT::NodeStatus::SUCCESS) {
            // This mission succeeded
            currently_executing_ = false;
            
            // Calculate reward
            double reward = calculateReward(current_state_, idx, true);
            
            // Update Q-table if learning
            if (learning_enabled_) {
                updateQValue(current_state_, current_order_, reward);
            }
            
            return BT::NodeStatus::SUCCESS;
        }
        else if (child_status == BT::NodeStatus::RUNNING) {
            // Child is still running, just return RUNNING without changing anything
            return BT::NodeStatus::RUNNING;
        }
        else {
            // Child failed, move to next one
            current_child_index_++;
            return tick(); // Recursively try the next child
        }
    }
    
    // Only reach here if we're not currently executing a mission sequence
    
    // Update parameters from ports
    getInput("learning_rate", learning_rate_);
    getInput("discount_factor", discount_factor_);
    getInput("exploration_rate", exploration_rate_);
    
    // Get current state
    current_state_ = getCurrentState();
    
    // Skip if no children
    if (children_nodes_.empty()) {
        return BT::NodeStatus::SUCCESS;
    }
    
    // Initialize if needed
    if (children_indices_.empty()) {
        for (size_t i = 0; i < children_nodes_.size(); ++i) {
            children_indices_.push_back(i);
        }
    }
    
    // Select mission order using epsilon-greedy policy
    bool exploring = false;
    if (learning_enabled_ && (std::uniform_real_distribution<>(0, 1)(rng_) < exploration_rate_)) {
        // Exploration: try a random permutation
        current_order_ = getRandomOrder();
        exploring = true;
    } else {
        // Exploitation: try the best order according to Q-values
        current_order_ = getBestOrder(current_state_);
    }
    
    // Generate explanation
    std::stringstream ss;
    ss << "Current State: " << current_state_.toString() << "\n";
    ss << "Decision: " << (exploring ? "EXPLORING with random order" : "EXPLOITING with best known order") << "\n";
    ss << missionOrderToString(current_order_) << "\n";
    
    if (!exploring) {
        ss << "Expected value: " << getQValue(current_state_, current_order_) << "\n";
        ss << "Alternative options considered:\n";
        auto top3 = getTopOrders(current_state_, 3);
        for (size_t i = 1; i < std::min(top3.size(), size_t(3)); ++i) {
            ss << "  " << missionOrderToString(top3[i].first) << " (value: " << top3[i].second << ")\n";
        }
    }
    
    explanation_ = ss.str();
    setOutput("explanation", explanation_);
    RCLCPP_INFO(g_node->get_logger(), "Starting new mission selection: %s", explanation_.c_str());
    
    // Mark that we're starting a new execution sequence
    currently_executing_ = true;
    current_child_index_ = 0;
    
    // Start with the first child by recursively calling tick
    return tick();
}

std::string LearningMissionSelector::missionOrderToString(const MissionOrder& order) {
    // FIXED: Correct mapping of indices to match the actual children in the behavior tree
    std::map<int, std::string> taskNames = {
        {0, "Pipeline Inspection"},
        {1, "Wreckage Inspection"},
        {2, "Return to Dock"}
    };
    
    std::stringstream ss;
    ss << "Mission Order: ";
    for (size_t i = 0; i < order.size(); ++i) {
        if (i > 0) ss << " -> ";
        if (order[i] < taskNames.size()) {
            ss << taskNames[order[i]];
        } else {
            ss << "Task" << order[i];
        }
    }
    return ss.str();
}

LearningMissionSelector::MissionState LearningMissionSelector::getCurrentState() {
    MissionState state;

    // Ensure ROS processes any pending messages
    rclcpp::spin_some(g_node);
    
    // Discretize battery level from the stored value
    if (current_battery_percentage_ < 30.0) state.battery_level = 0;      // Low
    else if (current_battery_percentage_ < 70.0) state.battery_level = 1; // Medium
    else state.battery_level = 2;                                        // High
    
    RCLCPP_INFO(g_node->get_logger(), "Using battery percentage: %.1f%% (level: %d)", 
               current_battery_percentage_, state.battery_level);
    
    // Check camera status based on time since last message
    auto time_since_last_camera_msg = (g_node->now() - last_camera_msg_time_).seconds();
    bool camera_status = camera_working_ && (time_since_last_camera_msg < camera_timeout_seconds_);
    
    state.camera_working = camera_status;
    
    RCLCPP_INFO(g_node->get_logger(), "Camera status: %s (last msg: %.1f seconds ago)", 
               state.camera_working ? "Working" : "Not working", 
               time_since_last_camera_msg);

    
    // Check sonar status based on time since last message
    auto time_since_last_sonar_msg = (g_node->now() - last_sonar_msg_time_).seconds();
    bool sonar_status = sonar_working_ && (time_since_last_sonar_msg < sonar_timeout_seconds_);
    
    state.sonar_working = sonar_status;
    
    RCLCPP_INFO(g_node->get_logger(), "Sonar status: %s (last msg: %.1f seconds ago)", 
               state.sonar_working ? "Working" : "Not working", 
               time_since_last_sonar_msg);
    
    // Use the distance to dock from the topic
    RCLCPP_INFO(g_node->get_logger(), "Distance to dock: %.2f m", current_distance_to_dock_);
    
    // Discretize distance
    if (current_distance_to_dock_ < 50.0) state.distance_to_dock = 0;      // Near
    else if (current_distance_to_dock_ < 150.0) state.distance_to_dock = 1; // Medium
    else state.distance_to_dock = 2;   
    
    return state;
}

LearningMissionSelector::MissionOrder LearningMissionSelector::getRandomOrder() {
    MissionOrder order = children_indices_;
    std::shuffle(order.begin(), order.end(), rng_);
    return order;
}

LearningMissionSelector::MissionOrder LearningMissionSelector::getBestOrder(const MissionState& state) {
    // FIXED: Force Return to Dock first when battery is low
    if (state.battery_level == 0) {
        MissionOrder safe_order = {2};  // Index 2 = Return to Dock
        // You can add the other missions after Return if desired
        if (children_nodes_.size() > 1) {
            safe_order.push_back(0);  // Pipeline Inspection
            safe_order.push_back(1);  // Wreckage Inspection
        }
        
        RCLCPP_WARN(g_node->get_logger(), "Low battery level! Forcing Return to Dock as first task");
        return safe_order;
    }
    
    // If state not in Q-table, return random order
    if (q_table_.find(state) == q_table_.end() || q_table_[state].empty()) {
        RCLCPP_INFO(g_node->get_logger(), "No Q-values for state %s, using random order", 
                   state.toString().c_str());
        return getRandomOrder();
    }
    
    // Find action with highest Q-value
    MissionOrder best_order;
    double best_value = -std::numeric_limits<double>::max();
    
    for (const auto& [order, value] : q_table_[state]) {
        if (value > best_value) {
            best_value = value;
            best_order = order;
        }
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Best order from Q-table: %s (Q=%.2f)", 
               missionOrderToString(best_order).c_str(), best_value);
    
    return best_order;
}

std::vector<std::pair<LearningMissionSelector::MissionOrder, double>> 
LearningMissionSelector::getTopOrders(const MissionState& state, int n) {
    std::vector<std::pair<MissionOrder, double>> orders;
    
    if (q_table_.find(state) != q_table_.end()) {
        for (const auto& [order, value] : q_table_[state]) {
            orders.push_back({order, value});
        }
        
        // Sort by value, descending
        std::sort(orders.begin(), orders.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });
    }
    
    // If less than n entries, add random orders
    while (orders.size() < static_cast<size_t>(n)) {
        MissionOrder random_order = getRandomOrder();
        orders.push_back({random_order, 0.0});
    }
    
    return orders;
}

double LearningMissionSelector::getQValue(const MissionState& state, const MissionOrder& order) {
    if (q_table_.find(state) != q_table_.end() && 
        q_table_[state].find(order) != q_table_[state].end()) {
        return q_table_[state][order];
    }
    return 0.0; // Default Q-value
}

void LearningMissionSelector::updateQValue(const MissionState& state, const MissionOrder& order, double reward) {
    // Simple Q-value update without next state (since we're treating each mission ordering as an episode)
    double old_q = getQValue(state, order);
    double new_q = (1 - learning_rate_) * old_q + learning_rate_ * reward;
    q_table_[state][order] = new_q;
    
    // Log the update
    RCLCPP_INFO(g_node->get_logger(), "Updated Q-value - State: %s, Order: %s", 
               state.toString().c_str(), missionOrderToString(order).c_str());
    RCLCPP_INFO(g_node->get_logger(), "   Old Value: %.2f → New Value: %.2f (Reward: %.2f)", 
               old_q, new_q, reward);
    
    // Save Q-table after every update
    saveQTable("/home/badawi/Desktop/blueyeROV_BT/src/blueye_bt/scripts/new_mission_q_table.txt");
    
    // Optional: Keep the counter for logging purposes only
    static int update_count = 0;
    update_count++;
    RCLCPP_INFO(g_node->get_logger(), "Q-table saved (update #%d)", update_count);
}

double LearningMissionSelector::calculateReward(const MissionState& state, size_t completed_task, bool success) {
    // Base reward for success or failure
    double reward = success ? 10.0 : -5.0;
    
    // FIXED: Adjust battery rewards to prioritize return to dock when battery is low
    if (state.battery_level == 0) {  // Low battery
        if (completed_task == 2) {  // Return to Dock (index 2)
            // Major reward for correctly returning when battery low
            reward += 20.0;
        } else {
            // Major penalty for not returning when battery low
            reward -= 15.0;
        }
    } else {
        // Extra reward for completing missions with medium battery (efficiency)
        if (success && state.battery_level == 1) {
            reward += 5.0;
        }
        
        // Extra reward for using working sensors appropriately
        if (success) {
            // Pipeline inspection (index 0) typically needs camera
            if (completed_task == 0 && state.camera_working) {
                reward += 3.0;
            }
            // Wreckage inspection (index 1) typically needs sonar
            if (completed_task == 1 && state.sonar_working) {
                reward += 3.0;
            }
        }
    }
    
    return reward;
}

void LearningMissionSelector::saveQTable(const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(g_node->get_logger(), "Could not open file for saving Q-table: %s", filename.c_str());
        return;
    }
    
    // Write format version
    file << "v1\n";
    
    // Write number of entries
    size_t entries = 0;
    for (const auto& [state, actions] : q_table_) {
        entries += actions.size();
    }
    file << entries << "\n";
    
    // Write each entry
    for (const auto& [state, actions] : q_table_) {
        for (const auto& [order, value] : actions) {
            // Write state
            file << state.battery_level << " "
                 << (state.camera_working ? 1 : 0) << " "
                 << (state.sonar_working ? 1 : 0) << " "
                 << state.distance_to_dock << " ";
            
            // Write order
            file << order.size() << " ";
            for (size_t idx : order) {
                file << idx << " ";
            }
            
            // Write Q-value
            file << value << "\n";
        }
    }
    
    RCLCPP_INFO(g_node->get_logger(), "Q-table saved to %s", filename.c_str());
}

void LearningMissionSelector::loadQTable(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_WARN(g_node->get_logger(), "Could not open file for loading Q-table: %s", filename.c_str());
        return;
    }
    
    try {
        // Read format version
        std::string version;
        file >> version;
        
        if (version != "v1") {
            RCLCPP_ERROR(g_node->get_logger(), "Unknown Q-table format: %s", version.c_str());
            return;
        }
        
        // Read number of entries
        size_t entries;
        file >> entries;
        
        // Clear existing table
        q_table_.clear();
        
        // Read each entry
        for (size_t i = 0; i < entries; ++i) {
            // Read state
            MissionState state;
            int camera_working, sonar_working;
            
            file >> state.battery_level >> camera_working >> sonar_working >> state.distance_to_dock;
            
            state.camera_working = camera_working == 1;
            state.sonar_working = sonar_working == 1;
            
            // Read order
            size_t order_size;
            file >> order_size;
            
            MissionOrder order(order_size);
            for (size_t j = 0; j < order_size; ++j) {
                file >> order[j];
            }
            
            // Read Q-value
            double q_value;
            file >> q_value;
            
            // FIXED: Validate order indices against children count
            bool valid_indices = true;
            for (size_t idx : order) {
                if (idx >= 3) {  // We expect 3 children max (0, 1, 2)
                    RCLCPP_WARN(g_node->get_logger(), 
                              "Invalid task index %zu in Q-table (max expected: 2). Skipping entry.", idx);
                    valid_indices = false;
                    break;
                }
            }
            
            // Store in Q-table if indices are valid
            if (valid_indices) {
                q_table_[state][order] = q_value;
            }
        }
        
        RCLCPP_INFO(g_node->get_logger(), "Q-table loaded from %s (%zu states)", filename.c_str(), q_table_.size());
        
        // Print a summary of the Q-table values
        RCLCPP_INFO(g_node->get_logger(), "Q-table summary:");
        for (const auto& [state, actions] : q_table_) {
            if (!actions.empty()) {
                // Find best action for this state
                double best_value = -std::numeric_limits<double>::max();
                MissionOrder best_order;
                
                for (const auto& [order, value] : actions) {
                    if (value > best_value) {
                        best_value = value;
                        best_order = order;
                    }
                }
                
                // Only print if we found a valid best action
                if (best_value > -std::numeric_limits<double>::max()) {
                    RCLCPP_INFO(g_node->get_logger(), "State: %s → Best order: %s (Q=%.2f)",
                            state.toString().c_str(),
                            missionOrderToString(best_order).c_str(),
                            best_value);
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Error loading Q-table: %s", e.what());
    }
}