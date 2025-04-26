#include "blueye_bt/controllers/altitude_controller.hpp"
#include <algorithm>
#include <vector>

AltitudeController::AltitudeController() : Node("altitude_controller"), 
    dvl_valid_(false), altitude_filter_initialized_(false) {
    
    // Parameters
    this->declare_parameter("target_altitude", 2.0);
    this->declare_parameter("altitude_control_enabled", false);
    this->declare_parameter("heave_speed", 0.3);
    this->declare_parameter("min_altitude", 0.5);
    this->declare_parameter("altitude_safety_margin", 0.2);
    
    target_altitude_ = this->get_parameter("target_altitude").as_double();
    altitude_control_enabled_ = this->get_parameter("altitude_control_enabled").as_bool();
    heave_speed_ = this->get_parameter("heave_speed").as_double();
    min_altitude_ = this->get_parameter("min_altitude").as_double();
    altitude_safety_margin_ = this->get_parameter("altitude_safety_margin").as_double();
    
    // Create subscribers
    dvl_sub_ = this->create_subscription<marine_acoustic_msgs::msg::Dvl>(
        "/blueye/dvl", 10,
        std::bind(&AltitudeController::dvlCallback, this, std::placeholders::_1));
        
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/blueye/ref_vel", 10,
        std::bind(&AltitudeController::cmdVelCallback, this, std::placeholders::_1));
    
    // Create publisher for modified velocity
    modified_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/blueye/altitude_ref_vel", 10);
    
    // Create service for enabling/disabling
    enable_service_ = this->create_service<std_srvs::srv::SetBool>(
        "/altitude_controller/enable",
        std::bind(&AltitudeController::enableAltitudeControlCallback, this, 
                 std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Altitude controller initialized. Target: %.2f m, Enabled: %s",
               target_altitude_, altitude_control_enabled_ ? "true" : "false");
}

void AltitudeController::dvlCallback(const marine_acoustic_msgs::msg::Dvl::SharedPtr msg) {
    // Store raw altitude for debugging
    float raw_altitude = msg->altitude;
    
    // Apply filtering to altitude measurements
    if (!altitude_filter_initialized_) {
        // Initialize filter with first valid measurement
        filtered_altitude_ = raw_altitude;
        for (size_t i = 0; i < ALTITUDE_BUFFER_SIZE; ++i) {
            altitude_buffer_.push_back(raw_altitude);
        }
        altitude_filter_initialized_ = true;
    } else {
        // Add new measurement to buffer
        altitude_buffer_.push_back(raw_altitude);
        if (altitude_buffer_.size() > ALTITUDE_BUFFER_SIZE) {
            altitude_buffer_.pop_front();
        }
        
        // Calculate median of buffer (more robust than mean)
        std::vector<float> sorted_altitudes(altitude_buffer_.begin(), altitude_buffer_.end());
        std::sort(sorted_altitudes.begin(), sorted_altitudes.end());
        float median_altitude = sorted_altitudes[sorted_altitudes.size() / 2];
        
        // Apply low-pass filter to median value for even smoother result
        const float alpha = 0.1; // Lower = smoother but more lag
        filtered_altitude_ = alpha * median_altitude + (1.0 - alpha) * filtered_altitude_;
    }
    
    // Use filtered altitude for control
    current_altitude_ = filtered_altitude_;
    
    last_dvl_time_ = this->now();
    dvl_valid_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "DVL update: raw=%.2f m, filtered=%.2f m", 
                raw_altitude, filtered_altitude_);
}

void AltitudeController::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // Check for parameter updates
    target_altitude_ = this->get_parameter("target_altitude").as_double();
    
    // Make a copy of the incoming velocity command
    auto modified_msg = std::make_unique<geometry_msgs::msg::TwistStamped>(*msg);
    
    // If altitude control is enabled, calculate and override the z component
    if (altitude_control_enabled_ && dvl_valid_) {
        auto time_since_dvl = this->now() - last_dvl_time_;
        if (time_since_dvl.seconds() <= 1.0) {
            // Calculate the altitude control command
            double z_vel = calculateAltitudeControl(msg->header.stamp);
            
            // Override the z velocity
            modified_msg->twist.linear.z = z_vel;
            
            RCLCPP_INFO(this->get_logger(), 
                      "Altitude control active: altitude=%.2f, target=%.2f, z_vel=%.2f",
                      current_altitude_, target_altitude_, z_vel);
        } else {
            RCLCPP_WARN(this->get_logger(), 
                       "DVL data too old (%.1f seconds), passing through original z command",
                       time_since_dvl.seconds());
        }
    } else if (altitude_control_enabled_ && !dvl_valid_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                            "Altitude control enabled but no valid DVL data received yet");
    }
    
    // Publish the modified command
    modified_cmd_vel_pub_->publish(std::move(modified_msg));
}

double AltitudeController::calculateAltitudeControl(const rclcpp::Time& current_time) {
    // Calculate altitude error (positive when altitude is too low)
    float altitude_err = target_altitude_ - current_altitude_;
    
    // Debug log the current state
    RCLCPP_DEBUG(this->get_logger(), "Altitude error calculation: target=%.2f, current=%.2f, error=%.2f",
               target_altitude_, current_altitude_, altitude_err);
    
    // Safety check
    if (current_altitude_ < min_altitude_ + altitude_safety_margin_ && altitude_err < 0) {
        RCLCPP_WARN(this->get_logger(),
                  "Altitude too low (%.2f m) - restricting downward movement", 
                  current_altitude_);
        altitude_err = std::max(0.0f, altitude_err);
    }
    
    // Limit error magnitude to prevent excessive speeds
    float max_error = 2.0f;  // Maximum error to consider for control
    altitude_err = std::max(std::min(altitude_err, max_error), -max_error);
    
    // Calculate z velocity from altitude error
    // Positive error → need to go UP → negative z velocity (NED frame)
    double z_vel = -altitude_err * heave_speed_ / max_error;
    
    // Ensure we respect max speed limits
    z_vel = std::max(std::min(z_vel, heave_speed_), -heave_speed_);
    
    return z_vel;
}

void AltitudeController::enableAltitudeControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    altitude_control_enabled_ = request->data;
    
    if (altitude_control_enabled_) {
        RCLCPP_INFO(this->get_logger(), "Altitude control enabled with target %.2f m", 
                   target_altitude_);
    } else {
        RCLCPP_INFO(this->get_logger(), "Altitude control disabled");
    }
    
    response->success = true;
    response->message = altitude_control_enabled_ ? 
                        "Altitude control enabled" : 
                        "Altitude control disabled";
}

// Add the main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AltitudeController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}