#include "gz_robot_monitors/geofence_monitor.h"
#include <cmath>  // For std::sqrt

namespace gz_robot_monitors
{
    GeofenceMonitor::GeofenceMonitor() : Node("geofence_monitor")
    {
        // Subscription to /blueye/usbl
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/blueye/usbl",
            10,
            std::bind(&GeofenceMonitor::on_usbl_message, this, std::placeholders::_1)
        );

        // Publisher for /blueye/monitor/is_within_geofence
        publisher_ = this->create_publisher<std_msgs::msg::Bool>(
            "/blueye/monitor/is_within_geofence",
            10
        );

        RCLCPP_DEBUG(this->get_logger(), "Geofence Monitor Node has started.");
    }

    void GeofenceMonitor::on_usbl_message(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Extract position
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double z = msg->pose.position.z;

        // Log received data
        RCLCPP_DEBUG(this->get_logger(), "Received USBL data: x=%.2f, y=%.2f, z=%.2f", x, y, z);

        // Calculate distance
        double distance = std::sqrt(x * x + y * y + z * z);

        // Check geofence condition
        std_msgs::msg::Bool status_msg;
        status_msg.data = distance <= geofence_threshold_;

        // Publish geofence status
        publisher_->publish(status_msg);

        // Log result
        RCLCPP_DEBUG(
            this->get_logger(),
            "Distance: %.2f, Status: %s",
            distance,
            status_msg.data ? "Within Geofence" : "Outside Geofence"
        );
    }
}  // namespace gz_robot_monitors
