#include "blueye_bt/control_nodes/learning_mission_selector.hpp"
#include <algorithm>
#include <numeric>
#include <limits>
#include <tuple>
#include <fstream>
#include <sstream>

// Define the static constants for mission state indices
const size_t LearningMissionSelector::STATE_TRANSIT_1;
const size_t LearningMissionSelector::STATE_PIPELINE;
const size_t LearningMissionSelector::STATE_TRANSIT_2;
const size_t LearningMissionSelector::STATE_WRECKAGE;
const size_t LearningMissionSelector::STATE_RETURN;

LearningMissionSelector::LearningMissionSelector(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ControlNode(name, config) {
    
    // Default parameters - increase exploration rate to encourage more randomness
    learning_rate_ = 0.1;
    discount_factor_ = 0.9;
    exploration_rate_ = 0.8;  // Increased to 0.8 to favor random exploration at the start

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

    // Comment out this line to start without loading the existing Q-table
    // loadQTable("/tmp/mission_q_table.txt");
    
    RCLCPP_INFO(g_node->get_logger(), "Learning Mission Selector initialized with fresh Q-table");
    RCLCPP_INFO(g_node->get_logger(), "Using high exploration rate (%.1f) to encourage random ordering", exploration_rate_);
}

BT::PortsList LearningMissionSelector::providedPorts() {
    return {
        BT::InputPort<double>("learning_rate", 0.1, "Alpha parameter for Q-learning"),
        BT::InputPort<double>("discount_factor", 0.9, "Gamma parameter for Q-learning"),
        BT::InputPort<double>("exploration_rate", 0.8, "Epsilon for exploration strategy"),
        BT::OutputPort<std::string>("explanation", "Explanation of decision making")
    };
}

BT::NodeStatus LearningMissionSelector::tick() {
    // If we're already executing a mission sequence, continue from where we left off
    if (currently_executing_) {
        // Get the current child we're executing
        if (current_child_index_ >= current_order_.size()) {
            // We've completed all middle states
            currently_executing_ = false;
            middle_states_completed_ = true;
            
            // Calculate reward for success
            double reward = calculateReward(current_state_, current_order_.back(), true);
            
            // Update Q-table
            if (learning_enabled_) {
                updateQValue(current_state_, current_order_, reward);
            }
            
            return BT::NodeStatus::SUCCESS;
        }
        
        size_t idx = current_order_[current_child_index_];
        BT::NodeStatus child_status = children_nodes_[idx]->executeTick();
        
        if (child_status == BT::NodeStatus::SUCCESS) {
            // This mission step succeeded, move to next one
            current_child_index_++;
            
            // If we have more steps, continue running
            if (current_child_index_ < current_order_.size()) {
                return BT::NodeStatus::RUNNING;
            } else {
                // We've completed all children in this order
                currently_executing_ = false;
                
                // Calculate reward for success of the whole sequence
                double reward = calculateReward(current_state_, current_order_.back(), true);
                
                // Update Q-table
                if (learning_enabled_) {
                    updateQValue(current_state_, current_order_, reward);
                }
                
                return BT::NodeStatus::SUCCESS;
            }
        }
        else if (child_status == BT::NodeStatus::RUNNING) {
            // Child is still running, just return RUNNING
            return BT::NodeStatus::RUNNING;
        }
        else {
            // Child failed, mission fails
            currently_executing_ = false;
            
            // Calculate reward for failure
            double reward = calculateReward(current_state_, current_order_[current_child_index_], false);
            
            // Update Q-table
            if (learning_enabled_) {
                updateQValue(current_state_, current_order_, reward);
            }
            
            return BT::NodeStatus::FAILURE;
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
    
    // Modified selection to prefer random exploration more initially
    bool exploring = false;
    // Always explore during first few runs, then use epsilon-greedy
    static int run_count = 0;
    if (run_count < 5 || (learning_enabled_ && (std::uniform_real_distribution<>(0, 1)(rng_) < exploration_rate_))) {
        // Exploration: try a random but valid permutation
        do {
            current_order_ = getRandomOrder();
        } while (!validateMissionOrder(current_order_));
        exploring = true;
        
        run_count++;
    } else {
        // Exploitation: try the best valid order according to Q-values
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
            if (validateMissionOrder(top3[i].first)) {
                ss << "  " << missionOrderToString(top3[i].first) << " (value: " << top3[i].second << ")\n";
            }
        }
    }
    
    explanation_ = ss.str();
    setOutput("explanation", explanation_);
    // Add this line to write to file
    std::ofstream outfile("/tmp/rl_explanation.txt");
    outfile << explanation_;
    outfile.close();

    RCLCPP_INFO(g_node->get_logger(), "Starting mission selection: %s", explanation_.c_str());

    // Mark that we're starting a new execution sequence
    currently_executing_ = true;
    current_child_index_ = 0;
    
    // Start the sequence - returning RUNNING since we're not done yet
    return BT::NodeStatus::RUNNING;
}

std::string LearningMissionSelector::missionOrderToString(const MissionOrder& order) {
    std::map<int, std::string> taskNames = {
        {STATE_TRANSIT_1, "Transit 1"},
        {STATE_PIPELINE, "Pipeline Inspection"},
        {STATE_TRANSIT_2, "Transit 2"},
        {STATE_WRECKAGE, "Wreckage Inspection"},
        {STATE_RETURN, "Homing (Return to Dock)"}
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
    else state.distance_to_dock = 2;                                       // Far
    
    return state;
}

LearningMissionSelector::MissionOrder LearningMissionSelector::getRandomOrder() {
    MissionOrder order = children_indices_;
    std::shuffle(order.begin(), order.end(), rng_);
    return order;
}

bool LearningMissionSelector::validateMissionOrder(const MissionOrder& order) {
    // Check for Transit_1 before Pipeline Inspection
    bool transit1_found = false;
    bool pipeline_found = false;
    
    // Check for Transit_2 before Wreckage Inspection
    bool transit2_found = false;
    bool wreckage_found = false;
    
    for (size_t idx : order) {
        if (idx == STATE_TRANSIT_1) transit1_found = true;
        if (idx == STATE_PIPELINE) {
            if (!transit1_found) return false; // Transit_1 must come before Pipeline
            pipeline_found = true;
        }
        
        if (idx == STATE_TRANSIT_2) transit2_found = true;
        if (idx == STATE_WRECKAGE) {
            if (!transit2_found) return false; // Transit_2 must come before Wreckage
            wreckage_found = true;
        }
    }
    
    return true;
}

LearningMissionSelector::MissionOrder LearningMissionSelector::getBestOrder(const MissionState& state) {
    // If state not in Q-table, return random valid order
    if (q_table_.find(state) == q_table_.end() || q_table_[state].empty()) {
        MissionOrder random_order;
        do {
            random_order = getRandomOrder();
        } while (!validateMissionOrder(random_order));
        return random_order;
    }
    
    // Find action with highest Q-value that passes validation
    MissionOrder best_order;
    double best_value = -std::numeric_limits<double>::max();
    
    for (const auto& [order, value] : q_table_[state]) {
        if (value > best_value && validateMissionOrder(order)) {
            best_value = value;
            best_order = order;
        }
    }
    
    // If no valid order found, generate a random valid one
    if (best_order.empty()) {
        do {
            best_order = getRandomOrder();
        } while (!validateMissionOrder(best_order));
    }
    
    return best_order;
}

std::vector<std::pair<LearningMissionSelector::MissionOrder, double>> 
LearningMissionSelector::getTopOrders(const MissionState& state, int n) {
    std::vector<std::pair<MissionOrder, double>> orders;
    
    if (q_table_.find(state) != q_table_.end()) {
        for (const auto& [order, value] : q_table_[state]) {
            // Only add valid orders
            if (validateMissionOrder(order)) {
                orders.push_back({order, value});
            }
        }
        
        // Sort by value, descending
        std::sort(orders.begin(), orders.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });
    }
    
    // If less than n entries, add random valid orders
    while (orders.size() < static_cast<size_t>(n)) {
        MissionOrder random_order;
        do {
            random_order = getRandomOrder();
        } while (!validateMissionOrder(random_order));
        
        // Check if this random order is already in the list
        bool exists = false;
        for (const auto& [existing_order, _] : orders) {
            if (existing_order == random_order) {
                exists = true;
                break;
            }
        }
        
        if (!exists) {
            orders.push_back({random_order, 0.0});
        }
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
    saveQTable("/tmp/mission_q_table.txt");
    
    // Optional: Keep the counter for logging purposes only
    static int update_count = 0;
    update_count++;
    RCLCPP_INFO(g_node->get_logger(), "Q-table saved (update #%d)", update_count);
}

double LearningMissionSelector::calculateReward(const MissionState& state, size_t completed_task, bool success) {
    // Base reward for success or failure
    double reward = success ? 10.0 : -5.0;
    
    // Extra reward for completing missions with low battery (efficiency)
    if (success && state.battery_level == 0) {
        reward += 5.0;
    }
    
    // Extra reward for using working sensors appropriately
    if (success) {
        // Transit points
        if ((completed_task == STATE_TRANSIT_1 || completed_task == STATE_TRANSIT_2)) {
            // Transit successfully completed
            reward += 1.0;
        }
        
        // Pipeline inspection typically needs camera
        if (completed_task == STATE_PIPELINE && state.camera_working) {
            reward += 3.0;
        }
        
        // Wreckage inspection typically needs sonar
        if (completed_task == STATE_WRECKAGE && state.sonar_working) {
            reward += 3.0;
        }
    }
    
    // Penalty for not returning to dock when battery is low
    if (state.battery_level == 0 && completed_task != STATE_RETURN) {
        reward -= 4.0;
    }
    
    // Extra reward for optimal ordering
    // If transit missions are completed early with high battery
    if ((completed_task == STATE_TRANSIT_1 || completed_task == STATE_TRANSIT_2) && 
        state.battery_level == 2) {
        reward += 2.0;
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
            
            // Store in Q-table
            q_table_[state][order] = q_value;
        }
        
        RCLCPP_INFO(g_node->get_logger(), "Q-table loaded from %s (%zu states)", filename.c_str(), q_table_.size());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(g_node->get_logger(), "Error loading Q-table: %s", e.what());
    }
}