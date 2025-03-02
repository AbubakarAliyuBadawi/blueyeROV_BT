// ================== Comments ==================
// frd: Front, Right, Down
// flu: Front, Left, Up
// ned: North, East, Down
// enu: East, North, Up
// ==============================================

#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node {
public:
  FramePublisher() : Node("tf2_blueye") {
    // ROS2 Parameters
    this->declare_parameter("model_name", "blueye");
    this->model_name = this->get_parameter("model_name")
                           .get_parameter_value()
                           .get<std::string>();

    // TF2 Broadcast Frames
    // tf_bluerov_frd_to_world_ned_bc_ =
    //     std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    tf_world_enu_to_ned_bc_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    tf_bluerov_flu_to_frd_bc_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    this->make_transforms();

    // ROS2 publishers and subscribers
    odom_enu_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->model_name + "/odometry_flu/gt", 10,
        std::bind(&FramePublisher::handle_bluerov_pose, this,
                  std::placeholders::_1));

    // imu_flu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    //     this->model_name + "/link/base_link/sensor/imu_sensor/imu", 10,
    //     std::bind(&FramePublisher::handle_imu_msg, this,
    //               std::placeholders::_1));

    imu_frd_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
        this->model_name + "/imu_frd", 10);

    odom_ned_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        this->model_name + "/odometry_frd/gt", 10);
  }

private:
  void make_transforms() {

    // timestamp
    auto time_stamp = this->get_clock()->now();

    tf_world_enu_to_ned.header.stamp = time_stamp;
    tf_world_enu_to_ned.header.frame_id = "world";
    tf_world_enu_to_ned.child_frame_id = "world_ned";

    tf_world_enu_to_ned.transform.translation.x = 0;
    tf_world_enu_to_ned.transform.translation.y = 0;
    tf_world_enu_to_ned.transform.translation.z = 0;
    tf_world_enu_to_ned.transform.rotation.x = 1.0 / sqrt(2.0);
    tf_world_enu_to_ned.transform.rotation.y = 1.0 / sqrt(2.0);
    tf_world_enu_to_ned.transform.rotation.z = 0;
    tf_world_enu_to_ned.transform.rotation.w = 0;

    tf_bluerov_flu_to_frd.header.stamp = time_stamp;
    tf_bluerov_flu_to_frd.header.frame_id = this->model_name + "_flu";
    tf_bluerov_flu_to_frd.child_frame_id = this->model_name + "_frd";

    tf_bluerov_flu_to_frd.transform.translation.x = 0;
    tf_bluerov_flu_to_frd.transform.translation.y = 0;
    tf_bluerov_flu_to_frd.transform.translation.z = 0;
    tf_bluerov_flu_to_frd.transform.rotation.x = 1;
    tf_bluerov_flu_to_frd.transform.rotation.y = 0;
    tf_bluerov_flu_to_frd.transform.rotation.z = 0;
    tf_bluerov_flu_to_frd.transform.rotation.w = 0;

    tf_world_enu_to_ned_bc_->sendTransform(tf_world_enu_to_ned);
    tf_bluerov_flu_to_frd_bc_->sendTransform(tf_bluerov_flu_to_frd);
  }

  void handle_bluerov_pose(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {

    // timestamp
    auto time_stamp = this->get_clock()->now();

    // transform from world enu to vehicle enu. Is this used anywhere?
    // tf_bluerov_frd_to_world_ned_bc_->sendTransform(tf_world_enu_to_bluerov_enu);

    // populate world enu to vehicle enu msg
    geometry_msgs::msg::TransformStamped tf_world_enu_to_bluerov_enu;
    tf_world_enu_to_bluerov_enu.header.stamp = time_stamp;
    tf_world_enu_to_bluerov_enu.header.frame_id = "world";
    tf_world_enu_to_bluerov_enu.child_frame_id = this->model_name + "_flu";
    tf_world_enu_to_bluerov_enu.transform.translation.x =
        msg->pose.pose.position.x;
    tf_world_enu_to_bluerov_enu.transform.translation.y =
        msg->pose.pose.position.y;
    tf_world_enu_to_bluerov_enu.transform.translation.z =
        msg->pose.pose.position.z;

    tf_world_enu_to_bluerov_enu.transform.rotation.x =
        msg->pose.pose.orientation.x;
    tf_world_enu_to_bluerov_enu.transform.rotation.y =
        msg->pose.pose.orientation.y;
    tf_world_enu_to_bluerov_enu.transform.rotation.z =
        msg->pose.pose.orientation.z;
    tf_world_enu_to_bluerov_enu.transform.rotation.w =
        msg->pose.pose.orientation.w;

    // transform pos_enu to pos_ned using tf_world_enu_to_bluerov_enu
    geometry_msgs::msg::Point pos_enu = msg->pose.pose.position;
    geometry_msgs::msg::Point pos_ned;

    tf2::doTransform(pos_enu, pos_ned, tf_world_enu_to_ned);

    // transform vel_flu to vel_frd
    geometry_msgs::msg::Vector3 vel_flu = msg->twist.twist.linear;
    geometry_msgs::msg::Vector3 vel_frd;

    tf2::doTransform(vel_flu, vel_frd, tf_bluerov_flu_to_frd);

    // transform ang_vel_flu to ang_vel_frd
    geometry_msgs::msg::Vector3 ang_vel_flu = msg->twist.twist.angular;
    geometry_msgs::msg::Vector3 ang_vel_frd;
    tf2::doTransform(ang_vel_flu, ang_vel_frd, tf_bluerov_flu_to_frd);

    // transform msg to tf2 quaternion type
    geometry_msgs::msg::Quaternion orientation_enu = msg->pose.pose.orientation;
    tf2::Quaternion q_enu_tf;
    tf2::convert(orientation_enu, q_enu_tf);

    // eigen quaternion from tf2 quaternion
    auto q_enu = Eigen::Quaterniond(q_enu_tf.getW(), q_enu_tf.getX(),
                                    q_enu_tf.getY(), q_enu_tf.getZ());

    // quaternion from enu to ned
    auto q_enu_to_ned =
        Eigen::Quaterniond(0.0, 1.0 / sqrt(2.0), 1.0 / sqrt(2.0), 0.0);

    // quaternion from frd to flu
    auto q_frd_to_flu = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0);

    // quaternion rotation from frd to ned, aka from body to ned.
    auto q_frd_to_ned = q_enu_to_ned * q_enu * q_frd_to_flu;

    // populate quaternion msg
    geometry_msgs::msg::Quaternion orientation_ned_frd;
    orientation_ned_frd.x = q_frd_to_ned.x();
    orientation_ned_frd.y = q_frd_to_ned.y();
    orientation_ned_frd.z = q_frd_to_ned.z();
    orientation_ned_frd.w = q_frd_to_ned.w();

    // populate odometry msg and publish to ROS
    nav_msgs::msg::Odometry odom_ned;
    odom_ned.header.stamp = time_stamp;
    odom_ned.header.frame_id = "world_ned";
    odom_ned.child_frame_id = this->model_name + "_frd";
    odom_ned.pose.pose.position = pos_ned;
    odom_ned.pose.pose.orientation = orientation_ned_frd;
    odom_ned.twist.twist.linear = vel_frd;
    odom_ned.twist.twist.angular = ang_vel_frd;

    odom_ned_pub_->publish(odom_ned);

  }

  void handle_imu_msg(const std::shared_ptr<sensor_msgs::msg::Imu> msg) {
    // timestamp
    auto time_stamp = this->get_clock()->now();

    // acceleration
    geometry_msgs::msg::Vector3 acc_flu = msg->linear_acceleration;
    geometry_msgs::msg::Vector3 acc_frd;
    tf2::doTransform(acc_flu, acc_frd, tf_bluerov_flu_to_frd);

    // angular velocity
    geometry_msgs::msg::Vector3 ang_vel_flu = msg->angular_velocity;
    geometry_msgs::msg::Vector3 ang_vel_frd;
    tf2::doTransform(ang_vel_flu, ang_vel_frd, tf_bluerov_flu_to_frd);

    // populate sensor msg and publish
    sensor_msgs::msg::Imu imu_frd;

    imu_frd.header.stamp = time_stamp;
    imu_frd.header.frame_id = this->model_name + "_frd";

    imu_frd.linear_acceleration = acc_frd;
    imu_frd.linear_acceleration_covariance =
        msg->linear_acceleration_covariance;

    imu_frd.angular_velocity = ang_vel_frd;
    imu_frd.angular_velocity_covariance = msg->angular_velocity_covariance;

    imu_frd_pub_->publish(imu_frd);
  }

  // tf2 transforms
  geometry_msgs::msg::TransformStamped tf_world_enu_to_ned;
  geometry_msgs::msg::TransformStamped tf_bluerov_flu_to_frd;

  // tf2 broadcasters
  //   std::unique_ptr<tf2_ros::TransformBroadcaster>
  //       tf_bluerov_frd_to_world_ned_bc_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_world_enu_to_ned_bc_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster>
      tf_bluerov_flu_to_frd_bc_;

  // ros2 publisher and subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_enu_sub_;
  //   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_flu_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_ned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_frd_pub_;

  // misc
  std::string model_name;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FramePublisher>());
  rclcpp::shutdown();
  return 0;
}

