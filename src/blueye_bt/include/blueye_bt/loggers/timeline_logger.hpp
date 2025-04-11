// include/blueye_bt/loggers/timeline_logger.hpp
#ifndef TIMELINE_LOGGER_HPP
#define TIMELINE_LOGGER_HPP

#include <behaviortree_cpp/loggers/abstract_logger.h>
#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <vector>
#include <string>
#include <chrono>

namespace blueye_bt {

class TimelineLogger : public BT::StatusChangeLogger
{
public:
    struct StateChange {
        std::chrono::steady_clock::time_point timestamp;
        std::string node_name;
        BT::NodeStatus prev_status;
        BT::NodeStatus status;
    };

    TimelineLogger(const BT::Tree& tree, rclcpp::Node::SharedPtr node)
        : StatusChangeLogger(tree.rootNode()), node_(node)
    {
        // Record start time
        start_time_ = std::chrono::steady_clock::now();
        
        // Create a timer to periodically save the data
        save_timer_ = node_->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&TimelineLogger::save_data, this));
    }

    // Override the pure virtual function from StatusChangeLogger
    void flush() override
    {
        save_data();
    }

    void callback(BT::Duration /*timestamp*/, const BT::TreeNode& node,
                  BT::NodeStatus prev_status, BT::NodeStatus status) override
    {
        StateChange change;
        change.timestamp = std::chrono::steady_clock::now();
        change.node_name = node.name();
        change.prev_status = prev_status;
        change.status = status;
        
        state_changes_.push_back(change);
    }

    void save_data()
    {
        std::string file_path = "/tmp/bt_timeline_data.csv";
        std::ofstream out(file_path);
        
        // Write header
        out << "timestamp,node_name,prev_status,status" << std::endl;
        
        // Write data
        for (const auto& change : state_changes_)
        {
            auto time_since_start = std::chrono::duration_cast<std::chrono::milliseconds>(
                change.timestamp - start_time_).count() / 1000.0;
                
            out << time_since_start << ","
                << change.node_name << ","
                << toStr(change.prev_status) << ","
                << toStr(change.status) << std::endl;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Saved timeline data to %s", file_path.c_str());
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::TimerBase::SharedPtr save_timer_;
    std::vector<StateChange> state_changes_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace blueye_bt

#endif // TIMELINE_LOGGER_HPP