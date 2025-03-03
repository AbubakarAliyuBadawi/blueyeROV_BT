#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mundus_mir_msgs/msg/battery_status.hpp>
#include <mundus_mir_msgs/msg/return_recommendation.hpp>
#include <cmath>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ReturnCalculator : public rclcpp::Node {
public:
    ReturnCalculator() : Node("return_calculator") {
        // Declare parameters
        this->declare_parameter("safety_margin", 30.0);
        this->declare_parameter("update_frequency", 10.0);
        this->declare_parameter("average_speed", 0.5);
        
        // Get parameters
        safety_margin_ = this->get_parameter("safety_margin").as_double();
        update_frequency_ = this->get_parameter("update_frequency").as_double();
        average_speed_ = this->get_parameter("average_speed").as_double();
        
        // Create subscriptions
        battery_sub_ = this->create_subscription<mundus_mir_msgs::msg::BatteryStatus>(
            "/blueye/battery",
            10,
            std::bind(&ReturnCalculator::batteryCallback, this, std::placeholders::_1)
        );
        
        distance_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/blueye/distance_to_dock",
            10,
            std::bind(&ReturnCalculator::distanceCallback, this, std::placeholders::_1)
        );
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/blueye/odometry_frd/gt",
            10,
            std::bind(&ReturnCalculator::odomCallback, this, std::placeholders::_1)
        );
        
        // Create publisher
        recommendation_pub_ = this->create_publisher<mundus_mir_msgs::msg::ReturnRecommendation>(
            "/blueye/return_recommendation",
            10
        );
        
        // Create timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
            std::bind(&ReturnCalculator::calculateRecommendation, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Return Calculator Node initialized");
    }

private:
    // Subscriptions
    rclcpp::Subscription<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr battery_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher
    rclcpp::Publisher<mundus_mir_msgs::msg::ReturnRecommendation>::SharedPtr recommendation_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double safety_margin_;
    double update_frequency_;
    double average_speed_;
    
    // State variables
    double total_distance_ = 0.0;
    geometry_msgs::msg::Point last_position_;
    bool has_last_position_ = false;
    rclcpp::Time start_time_;
    rclcpp::Time last_movement_time_;
    bool is_moving_ = false;
    double movement_threshold_ = 0.05;  // m/s
    
    // Battery tracking
    double initial_battery_ = -1.0;
    double current_battery_ = 100.0;
    double battery_per_meter_ = -1.0;
    double hover_drain_rate_ = -1.0;
    
    // Hover tracking
    double hover_start_battery_ = -1.0;
    rclcpp::Time hover_start_time_;
    double hover_duration_ = 0.0;
    double battery_used_hovering_ = 0.0;
    
    // Current state
    double current_distance_ = 0.0;
    double current_speed_ = 0.0;
    
    void batteryCallback(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
        auto current_time = this->now();
        
        // Initialize battery tracking
        if (initial_battery_ < 0) {
            initial_battery_ = msg->state_of_charge * 100.0;
            start_time_ = current_time;
        }
        
        current_battery_ = msg->state_of_charge * 100.0;
        
        // Update hover drain calculations if not moving
        if (!is_moving_) {
            if (hover_start_battery_ < 0) {
                hover_start_battery_ = current_battery_;
                hover_start_time_ = current_time;
            } else {
                double hover_time = (current_time - hover_start_time_).seconds();
                if (hover_time > 0) {
                    double hover_drain = (hover_start_battery_ - current_battery_) / hover_time;
                    if (hover_drain_rate_ < 0) {
                        hover_drain_rate_ = hover_drain;
                    } else {
                        // Running average
                        hover_drain_rate_ = 0.9 * hover_drain_rate_ + 0.1 * hover_drain;
                    }
                }
            }
        }
    }
    
    void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_distance_ = msg->data;
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto current_time = this->now();
        auto current_pos = msg->pose.pose.position;
        
        // Calculate speed
        auto vel = msg->twist.twist.linear;
        current_speed_ = std::sqrt(vel.x*vel.x + vel.y*vel.y + vel.z*vel.z);
        
        // Determine if ROV is moving
        is_moving_ = current_speed_ > movement_threshold_;
        
        // Initialize position tracking
        if (!has_last_position_) {
            last_position_ = current_pos;
            last_movement_time_ = current_time;
            has_last_position_ = true;
            return;
        }
        
        // Calculate distance traveled
        double distance = std::sqrt(
            std::pow(current_pos.x - last_position_.x, 2) +
            std::pow(current_pos.y - last_position_.y, 2) +
            std::pow(current_pos.z - last_position_.z, 2)
        );
        
        if (is_moving_) {
            total_distance_ += distance;
            // Calculate battery per meter if we've moved a significant distance
            if (total_distance_ > 1.0 && initial_battery_ >= 0) {
                double battery_used = initial_battery_ - current_battery_;
                battery_per_meter_ = battery_used / total_distance_;
            }
        }
        
        last_position_ = current_pos;
        last_movement_time_ = current_time;
    }
    
    double estimateReturnEnergy() {
        if (battery_per_meter_ < 0 || hover_drain_rate_ < 0) {
            return 0.0;
        }
        
        // Add coefficient to reduce energy estimate
        double consumption_coefficient = 0.4;
        
        // Calculate travel energy
        double travel_energy = current_distance_ * battery_per_meter_ * consumption_coefficient;
        
        // Calculate hover energy during return
        double estimated_return_time = current_distance_ / average_speed_;
        double hover_energy = hover_drain_rate_ * estimated_return_time * consumption_coefficient;
        
        // Total energy needed
        return travel_energy + hover_energy;
    }
    
    void calculateRecommendation() {
        auto msg = std::make_unique<mundus_mir_msgs::msg::ReturnRecommendation>();
        msg->stamp = this->now();
        
        // Current state
        msg->current_battery_level = current_battery_;
        msg->distance_to_dock = current_distance_;
        msg->current_speed = current_speed_;
        
        // Calculate consumption rates
        if (battery_per_meter_ >= 0) {
            msg->current_consumption_rate = battery_per_meter_;
        }
        
        // Calculate estimates
        msg->estimated_return_energy = estimateReturnEnergy();
        msg->estimated_time_to_return = (current_distance_ > 0) ? 
            (current_distance_ / average_speed_) : 0.0;
        
        // Calculate minimum battery needed with safety margin
        double safety_margin_decimal = safety_margin_ / 100.0;
        msg->minimum_battery_needed = msg->estimated_return_energy * (1 + safety_margin_decimal);
        
        // Set safety margins
        msg->safety_margin_percent = safety_margin_;
        msg->battery_safety_threshold = 20.0;  // Minimum 20% battery level
        
        msg->should_return = (
            msg->current_battery_level <= msg->minimum_battery_needed ||
            msg->current_battery_level <= msg->battery_safety_threshold
        );
        
        // Publish recommendation
        recommendation_pub_->publish(std::move(msg));
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ReturnCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}