#ifndef GZ_ROBOT_MONITORS__GEOFENCE_MONITOR_H_
#define GZ_ROBOT_MONITORS__GEOFENCE_MONITOR_H_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gz_robot_monitors
{
    class GeofenceMonitor : public rclcpp::Node
    {
    public:
        
        GeofenceMonitor();

    private:

        void on_usbl_message(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;

        const double geofence_threshold_ = 400.0;
    };
}  // namespace gz_robot_monitors

#endif  // GZ_ROBOT_MONITORS__GEOFENCE_MONITOR_H_
