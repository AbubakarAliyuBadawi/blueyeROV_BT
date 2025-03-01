#ifndef ROV_MONITOR_GUI_HPP
#define ROV_MONITOR_GUI_HPP

#include <QMainWindow>
#include <QFrame>
#include <QWidget>
#include <QTimer>
#include <QLabel>
#include <QProgressBar>
#include <QComboBox>
#include <QPushButton>
#include <QString>
#include <QGroupBox>
#include <QGridLayout>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPainter>
#include <QPen>
#include <QMessageBox>
#include <QApplication>

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mundus_mir_msgs/msg/battery_status.hpp>
#include <mundus_mir_msgs/msg/return_recommendation.hpp>
#include <mundus_mir_msgs/srv/mission_manager.hpp>
#include <mundus_mir_msgs/srv/get_waypoint_status.hpp>
#include <mundus_mir_msgs/srv/get_waypoints.hpp>
#include <mundus_mir_msgs/srv/clear_waypoints.hpp>
#include <mundus_mir_msgs/srv/run_waypoint_controller.hpp>

// Visual display for ROV position
class RovPositionDisplay : public QFrame {
    Q_OBJECT
public:
    explicit RovPositionDisplay(QWidget *parent = nullptr);
    void updatePosition(double x, double y, double heading);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    double x_;
    double y_;
    double heading_;
};

// Visual display for mission status
class MissionStatusDisplay : public QFrame {
    Q_OBJECT
public:
    explicit MissionStatusDisplay(QWidget *parent = nullptr);
    void updateStatus(const std::string& statusCode);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    bool controllerRunning_;
    bool goingToWaypoint_;
    int waypointCount_;
    std::string currentWaypointData_;
};

// Main GUI class
class RovMonitorGui : public QMainWindow {
    Q_OBJECT
public:
    explicit RovMonitorGui(std::shared_ptr<rclcpp::Node> node, QWidget *parent = nullptr);
    ~RovMonitorGui();

private slots:
    void updateStatus();
    void switchMission();
    void toggleWaypointController();
    void clearWaypoints();

private:
    void setupUi();
    void setupRosSubscribers();
    void setupRosClients();

    void updatePositionDisplay(const nav_msgs::msg::Odometry::SharedPtr msg);
    void updateBatteryDisplay(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg);
    void updateReturnRecommendationDisplay(const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg);
    void requestWaypointStatus();
    void updateMissionStatus(const std::string& status);

    // ROS-related members
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscription_;
    rclcpp::Subscription<mundus_mir_msgs::msg::BatteryStatus>::SharedPtr batterySubscription_;
    rclcpp::Subscription<mundus_mir_msgs::msg::ReturnRecommendation>::SharedPtr returnRecommendationSubscription_;

    rclcpp::Client<mundus_mir_msgs::srv::MissionManager>::SharedPtr missionManagerClient_;
    rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedPtr waypointStatusClient_;
    rclcpp::Client<mundus_mir_msgs::srv::GetWaypoints>::SharedPtr getWaypointsClient_;
    rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedPtr clearWaypointsClient_;
    rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedPtr runWaypointControllerClient_;

    // UI components
    QComboBox *missionSelector_;
    QPushButton *runControllerButton_;
    QProgressBar *batteryLevelBar_;
    QLabel *voltageValue_;
    QLabel *currentValue_;
    QLabel *timeRemainingValue_;
    QLabel *returnRecommendationValue_;
    QLabel *posXValue_;
    QLabel *posYValue_;
    QLabel *depthValue_;
    QLabel *headingValue_;
    QLabel *waypoints_;
    RovPositionDisplay *positionDisplay_;
    MissionStatusDisplay *missionStatusDisplay_;
    QTimer *updateTimer_;
};

#endif // ROV_MONITOR_GUI_HPP
