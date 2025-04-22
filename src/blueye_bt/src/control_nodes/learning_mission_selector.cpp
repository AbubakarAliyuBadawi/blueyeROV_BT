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
    loadQTable("/tmp/mission_q_table.txt");
    
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
    RCLCPP_INFO(g_node->get_logger(), "Mission selection: %s", explanation_.c_str());
    
    // Now execute children in the selected order
    MissionOrder execution_sequence = current_order_;
    MissionState starting_state = current_state_;
    
    // Try each mission in the determined order
    for (size_t idx : execution_sequence) {
        if (idx >= children_nodes_.size()) {
            RCLCPP_ERROR(g_node->get_logger(), "Invalid child index %zu (max: %zu)", idx, children_nodes_.size() - 1);
            continue;
        }
        
        BT::NodeStatus child_status = children_nodes_[idx]->executeTick();
        
        if (child_status == BT::NodeStatus::SUCCESS) {
            // This mission succeeded
            
            // Calculate reward
            double reward = calculateReward(starting_state, idx, true);
            
            // Update Q-table if learning
            if (learning_enabled_) {
                updateQValue(starting_state, current_order_, reward);
            }
            
            return BT::NodeStatus::SUCCESS;
        }
        else if (child_status == BT::NodeStatus::RUNNING) {
            // Child is still running
            return BT::NodeStatus::RUNNING;
        }
        // If child returned FAILURE, try the next one
    }
    
    // All children failed
    double reward = calculateReward(starting_state, execution_sequence.back(), false);
    
    // Update Q-table if learning
    if (learning_enabled_) {
        updateQValue(starting_state, current_order_, reward);
    }
    
    return BT::NodeStatus::FAILURE;
}

std::string LearningMissionSelector::missionOrderToString(const MissionOrder& order) {
    std::map<int, std::string> taskNames = {
        {0, "Undocking"},
        {1, "Pipeline Inspection"},
        {2, "Wreckage Inspection"},
        {3, "Return to Dock"}
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
    // If state not in Q-table, return random order
    if (q_table_.find(state) == q_table_.end() || q_table_[state].empty()) {
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
    
    // Save Q-table occasionally (e.g., after significant updates)
    static int update_count = 0;
    if (++update_count % 10 == 0) {
        saveQTable("/tmp/mission_q_table.txt");
    }
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
        // Pipeline inspection typically needs camera
        if (completed_task == 1 && state.camera_working) {
            reward += 3.0;
        }
        // Wreckage inspection typically needs sonar
        if (completed_task == 2 && state.sonar_working) {
            reward += 3.0;
        }
    }
    
    // Penalty for not returning to dock when battery is low
    if (state.battery_level == 0 && completed_task != 3) {
        reward -= 4.0;
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