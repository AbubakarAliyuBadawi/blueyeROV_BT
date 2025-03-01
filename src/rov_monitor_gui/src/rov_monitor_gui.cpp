#include "rov_monitor_gui/rov_monitor_gui.hpp"

#include <cmath>
#include <sstream>
#include <thread>
#include <QStatusBar>

// -------------------------
// RovPositionDisplay Implementation
// -------------------------
RovPositionDisplay::RovPositionDisplay(QWidget *parent)
    : QFrame(parent), x_(0), y_(0), heading_(0) {
    setFrameStyle(QFrame::Box | QFrame::Sunken);
    setLineWidth(2);
}

void RovPositionDisplay::updatePosition(double x, double y, double heading) {
    x_ = x;
    y_ = y;
    heading_ = heading;
    update();  // Trigger redraw
}

void RovPositionDisplay::paintEvent(QPaintEvent *event) {
    QFrame::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    int centerX = width() / 2;
    int centerY = height() / 2;

    // Draw coordinate system
    painter.setPen(QPen(Qt::gray, 1, Qt::DashLine));
    painter.drawLine(0, centerY, width(), centerY);  // X-axis
    painter.drawLine(centerX, 0, centerX, height());   // Y-axis

    // Calculate ROV position (with scaling)
    double scale = 5.0;  // meters per 100 pixels
    int rovX = centerX + static_cast<int>(x_ * 100.0 / scale);
    int rovY = centerY - static_cast<int>(y_ * 100.0 / scale);  // Inverted Y-axis

    // Draw ROV (as a triangle)
    painter.setPen(QPen(Qt::black, 2));
    painter.setBrush(Qt::blue);
    painter.save();
    painter.translate(rovX, rovY);
    painter.rotate(heading_);
    const int size = 10;
    QPolygon triangle;
    triangle << QPoint(2 * size, 0) << QPoint(-size, -size) << QPoint(-size, size);
    painter.drawPolygon(triangle);
    painter.restore();

    // Draw scale indicator
    painter.setPen(QPen(Qt::black, 1));
    int scaleLength = static_cast<int>(100.0 / scale);  // 1 meter in pixels
    painter.drawLine(10, height() - 10, 10 + scaleLength, height() - 10);
    painter.drawText(10, height() - 15, QString("1m"));
}

// -------------------------
// MissionStatusDisplay Implementation
// -------------------------
MissionStatusDisplay::MissionStatusDisplay(QWidget *parent)
    : QFrame(parent), controllerRunning_(false), goingToWaypoint_(false),
      waypointCount_(0), currentWaypointData_("") {
    setFrameStyle(QFrame::Box | QFrame::Sunken);
    setLineWidth(2);
}

void MissionStatusDisplay::updateStatus(const std::string& statusCode) {
    std::istringstream ss(statusCode);
    std::string line;
    
    while (std::getline(ss, line)) {
        if (line.find("Controller running:") != std::string::npos) {
            controllerRunning_ = (line.find("1") != std::string::npos);
        } else if (line.find("Amount of waypoints:") != std::string::npos) {
            size_t pos = line.find_last_of(' ');
            if (pos != std::string::npos) {
                waypointCount_ = std::stoi(line.substr(pos + 1));
            }
        } else if (line.find("Currently going to waypoint:") != std::string::npos) {
            goingToWaypoint_ = (line.find("1") != std::string::npos);
        } else if (line.find("Current waypoint:") != std::string::npos ||
                   line.find("Station Keep On Position:") != std::string::npos) {
            currentWaypointData_ = line;
        }
    }
    
    update();  // Trigger redraw
}

void MissionStatusDisplay::paintEvent(QPaintEvent *event) {
    QFrame::paintEvent(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    const int margin = 10;
    const int textHeight = 20;

    // Draw controller status indicator
    painter.setPen(controllerRunning_ ? Qt::green : Qt::red);
    painter.setBrush(controllerRunning_ ? Qt::green : Qt::red);
    painter.drawEllipse(margin, margin, 10, 10);
    painter.setPen(Qt::black);
    painter.drawText(margin + 15, margin + textHeight / 2 + 5,
                     QString("Controller: %1").arg(controllerRunning_ ? "Running" : "Stopped"));

    // Draw waypoint info
    QString waypointInfo = (waypointCount_ > 0)
        ? QString("Waypoints: %1").arg(waypointCount_)
        : "No waypoints defined";
    painter.drawText(margin, margin + textHeight + 10, width() - 2 * margin, textHeight,
                     Qt::AlignLeft, waypointInfo);

    // Draw current activity
    QString activityInfo;
    if (controllerRunning_) {
        activityInfo = goingToWaypoint_ ? "Navigating to waypoint" : "Station keeping";
    } else {
        activityInfo = "Idle";
    }
    painter.drawText(margin, margin + 2 * textHeight + 20, width() - 2 * margin, textHeight,
                     Qt::AlignLeft, activityInfo);

    // Draw current waypoint/position data
    if (!currentWaypointData_.empty()) {
        painter.drawText(margin, margin + 3 * textHeight + 30, width() - 2 * margin, textHeight,
                         Qt::AlignLeft, QString::fromStdString(currentWaypointData_));
    }
}

// -------------------------
// RovMonitorGui Implementation
// -------------------------
RovMonitorGui::RovMonitorGui(std::shared_ptr<rclcpp::Node> node, QWidget *parent)
    : QMainWindow(parent), node_(node) {
    setWindowTitle("ROV Mission Monitor");
    setupUi();
    setupRosSubscribers();
    setupRosClients();

    updateTimer_ = new QTimer(this);
    connect(updateTimer_, &QTimer::timeout, this, &RovMonitorGui::updateStatus);
    updateTimer_->start(500); // 500ms update interval
}

RovMonitorGui::~RovMonitorGui() {
    // Child widgets are automatically cleaned up by Qt.
}

void RovMonitorGui::setupUi() {
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QVBoxLayout *mainLayout = new QVBoxLayout(centralWidget);

    // Mission control section
    QGroupBox *missionControlGroup = new QGroupBox("Mission Control", centralWidget);
    QGridLayout *missionControlLayout = new QGridLayout(missionControlGroup);
    QLabel *missionLabel = new QLabel("Mission Type:", missionControlGroup);
    missionSelector_ = new QComboBox(missionControlGroup);
    missionSelector_->addItem("Pipeline");
    missionSelector_->addItem("Lawnmower");
    missionSelector_->addItem("None");

    QPushButton *switchMissionButton = new QPushButton("Switch Mission", missionControlGroup);
    connect(switchMissionButton, &QPushButton::clicked, this, &RovMonitorGui::switchMission);

    runControllerButton_ = new QPushButton("Start Controller", missionControlGroup);
    connect(runControllerButton_, &QPushButton::clicked, this, &RovMonitorGui::toggleWaypointController);

    QPushButton *clearWaypointsButton = new QPushButton("Clear Waypoints", missionControlGroup);
    connect(clearWaypointsButton, &QPushButton::clicked, this, &RovMonitorGui::clearWaypoints);

    missionControlLayout->addWidget(missionLabel, 0, 0);
    missionControlLayout->addWidget(missionSelector_, 0, 1);
    missionControlLayout->addWidget(switchMissionButton, 0, 2);
    missionControlLayout->addWidget(runControllerButton_, 1, 0);
    missionControlLayout->addWidget(clearWaypointsButton, 1, 1);

    // Battery status section
    QGroupBox *batteryGroup = new QGroupBox("Battery Status", centralWidget);
    QGridLayout *batteryLayout = new QGridLayout(batteryGroup);
    QLabel *batteryLevelLabel = new QLabel("Charge:", batteryGroup);
    batteryLevelBar_ = new QProgressBar(batteryGroup);
    batteryLevelBar_->setRange(0, 100);
    batteryLevelBar_->setValue(100);
    batteryLevelBar_->setFormat("%p%");
    QLabel *voltageLabel = new QLabel("Voltage:", batteryGroup);
    voltageValue_ = new QLabel("0.0 V", batteryGroup);
    QLabel *currentLabel = new QLabel("Current:", batteryGroup);
    currentValue_ = new QLabel("0.0 A", batteryGroup);
    QLabel *timeRemainingLabel = new QLabel("Time Remaining:", batteryGroup);
    timeRemainingValue_ = new QLabel("N/A", batteryGroup);
    QLabel *returnRecommendationLabel = new QLabel("Return:", batteryGroup);
    returnRecommendationValue_ = new QLabel("Safe", batteryGroup);
    returnRecommendationValue_->setStyleSheet("color: green;");

    batteryLayout->addWidget(batteryLevelLabel, 0, 0);
    batteryLayout->addWidget(batteryLevelBar_, 0, 1, 1, 3);
    batteryLayout->addWidget(voltageLabel, 1, 0);
    batteryLayout->addWidget(voltageValue_, 1, 1);
    batteryLayout->addWidget(currentLabel, 1, 2);
    batteryLayout->addWidget(currentValue_, 1, 3);
    batteryLayout->addWidget(timeRemainingLabel, 2, 0);
    batteryLayout->addWidget(timeRemainingValue_, 2, 1);
    batteryLayout->addWidget(returnRecommendationLabel, 2, 2);
    batteryLayout->addWidget(returnRecommendationValue_, 2, 3);

    // Position & orientation section
    QGroupBox *positionGroup = new QGroupBox("Position & Orientation", centralWidget);
    QGridLayout *positionLayout = new QGridLayout(positionGroup);
    QLabel *posXLabel = new QLabel("X:", positionGroup);
    posXValue_ = new QLabel("0.0 m", positionGroup);
    QLabel *posYLabel = new QLabel("Y:", positionGroup);
    posYValue_ = new QLabel("0.0 m", positionGroup);
    QLabel *depthLabel = new QLabel("Depth:", positionGroup);
    depthValue_ = new QLabel("0.0 m", positionGroup);
    QLabel *headingLabel = new QLabel("Heading:", positionGroup);
    headingValue_ = new QLabel("0.0°", positionGroup);
    positionDisplay_ = new RovPositionDisplay(positionGroup);
    positionDisplay_->setMinimumSize(200, 200);

    positionLayout->addWidget(posXLabel, 0, 0);
    positionLayout->addWidget(posXValue_, 0, 1);
    positionLayout->addWidget(posYLabel, 1, 0);
    positionLayout->addWidget(posYValue_, 1, 1);
    positionLayout->addWidget(depthLabel, 2, 0);
    positionLayout->addWidget(depthValue_, 2, 1);
    positionLayout->addWidget(headingLabel, 3, 0);
    positionLayout->addWidget(headingValue_, 3, 1);
    positionLayout->addWidget(positionDisplay_, 0, 2, 4, 1);

    // Mission status section
    QGroupBox *missionStatusGroup = new QGroupBox("Mission Status", centralWidget);
    QVBoxLayout *missionStatusLayout = new QVBoxLayout(missionStatusGroup);
    missionStatusDisplay_ = new MissionStatusDisplay(missionStatusGroup);
    missionStatusDisplay_->setMinimumSize(400, 100);
    waypoints_ = new QLabel("No waypoints defined", missionStatusGroup);
    waypoints_->setWordWrap(true);
    missionStatusLayout->addWidget(missionStatusDisplay_);
    missionStatusLayout->addWidget(waypoints_);

    // Arrange sections in the main layout
    QHBoxLayout *topRowLayout = new QHBoxLayout();
    topRowLayout->addWidget(missionControlGroup);
    topRowLayout->addWidget(batteryGroup);
    QHBoxLayout *middleRowLayout = new QHBoxLayout();
    middleRowLayout->addWidget(positionGroup);
    mainLayout->addLayout(topRowLayout);
    mainLayout->addLayout(middleRowLayout);
    mainLayout->addWidget(missionStatusGroup);

    statusBar()->showMessage("Ready");
    resize(800, 600);
}

void RovMonitorGui::setupRosSubscribers() {
    odomSubscription_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/blueye/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            updatePositionDisplay(msg);
        });

    batterySubscription_ = node_->create_subscription<mundus_mir_msgs::msg::BatteryStatus>(
        "/blueye/battery", 10,
        [this](const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
            updateBatteryDisplay(msg);
        });

    returnRecommendationSubscription_ = node_->create_subscription<mundus_mir_msgs::msg::ReturnRecommendation>(
        "/blueye/return_recommendation", 10,
        [this](const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg) {
            updateReturnRecommendationDisplay(msg);
        });
}

void RovMonitorGui::setupRosClients() {
    missionManagerClient_ = node_->create_client<mundus_mir_msgs::srv::MissionManager>("/blueye/mission_manager");
    waypointStatusClient_ = node_->create_client<mundus_mir_msgs::srv::GetWaypointStatus>("/blueye/get_waypoint_status");
    getWaypointsClient_ = node_->create_client<mundus_mir_msgs::srv::GetWaypoints>("/blueye/get_waypoints");
    clearWaypointsClient_ = node_->create_client<mundus_mir_msgs::srv::ClearWaypoints>("/blueye/clear_waypoints");
    runWaypointControllerClient_ = node_->create_client<mundus_mir_msgs::srv::RunWaypointController>("/blueye/run_waypoint_controller");
}

void RovMonitorGui::updatePositionDisplay(const nav_msgs::msg::Odometry::SharedPtr msg) {
    posXValue_->setText(QString("%1 m").arg(msg->pose.pose.position.x, 0, 'f', 2));
    posYValue_->setText(QString("%1 m").arg(msg->pose.pose.position.y, 0, 'f', 2));
    depthValue_->setText(QString("%1 m").arg(msg->pose.pose.position.z, 0, 'f', 2));

    const auto& q = msg->pose.pose.orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp) * 180.0 / M_PI;
    headingValue_->setText(QString("%1°").arg(yaw, 0, 'f', 1));

    positionDisplay_->updatePosition(msg->pose.pose.position.x,
                                       msg->pose.pose.position.y,
                                       yaw);
}

void RovMonitorGui::updateBatteryDisplay(const mundus_mir_msgs::msg::BatteryStatus::SharedPtr msg) {
    int batteryPercent = static_cast<int>(msg->state_of_charge * 100.0);
    batteryLevelBar_->setValue(batteryPercent);
    if (batteryPercent < 20) {
        batteryLevelBar_->setStyleSheet("QProgressBar::chunk { background-color: red; }");
    } else if (batteryPercent < 40) {
        batteryLevelBar_->setStyleSheet("QProgressBar::chunk { background-color: orange; }");
    } else {
        batteryLevelBar_->setStyleSheet("QProgressBar::chunk { background-color: green; }");
    }
    voltageValue_->setText(QString("%1 V").arg(msg->total_voltage, 0, 'f', 1));
    currentValue_->setText(QString("%1 A").arg(msg->current, 0, 'f', 1));

    if (msg->average_time_to_empty > 0) {
        int minutes = msg->average_time_to_empty / 60;
        int seconds = msg->average_time_to_empty % 60;
        timeRemainingValue_->setText(QString("%1:%2").arg(minutes).arg(seconds, 2, 10, QChar('0')));
    } else {
        timeRemainingValue_->setText("N/A");
    }
}

void RovMonitorGui::updateReturnRecommendationDisplay(const mundus_mir_msgs::msg::ReturnRecommendation::SharedPtr msg) {
    if (msg->should_return) {
        returnRecommendationValue_->setText("Return Now");
        returnRecommendationValue_->setStyleSheet("color: red; font-weight: bold;");
    } else {
        returnRecommendationValue_->setText("Safe");
        returnRecommendationValue_->setStyleSheet("color: green;");
    }
}

void RovMonitorGui::requestWaypointStatus() {
    auto request = std::make_shared<mundus_mir_msgs::srv::GetWaypointStatus::Request>();
    auto result_future = waypointStatusClient_->async_send_request(
        request,
        [this](rclcpp::Client<mundus_mir_msgs::srv::GetWaypointStatus>::SharedFuture future) {
            auto response = future.get();
            if (response->accepted) {
                updateMissionStatus(response->status_code);
            }
        });

    auto waypoints_request = std::make_shared<mundus_mir_msgs::srv::GetWaypoints::Request>();
    auto waypoints_future = getWaypointsClient_->async_send_request(
        waypoints_request,
        [this](rclcpp::Client<mundus_mir_msgs::srv::GetWaypoints>::SharedFuture future) {
            auto response = future.get();
            if (response->accepted) {
                waypoints_->setText(QString::fromStdString(response->waypoints));
            }
        });
}

void RovMonitorGui::updateMissionStatus(const std::string& status) {
    missionStatusDisplay_->updateStatus(status);
}

void RovMonitorGui::updateStatus() {
    requestWaypointStatus();
}

void RovMonitorGui::switchMission() {
    QString missionType = missionSelector_->currentText().toLower();
    bool force = false;
    auto request = std::make_shared<mundus_mir_msgs::srv::MissionManager::Request>();
    request->mission_type = missionType.toStdString();
    request->force_switch = force;
    
    auto result_future = missionManagerClient_->async_send_request(
        request,
        [this](rclcpp::Client<mundus_mir_msgs::srv::MissionManager>::SharedFuture future) {
            auto response = future.get();
            if (response->success) {
                statusBar()->showMessage(
                    QString("Mission switched to: %1").arg(QString::fromStdString(response->current_mission)),
                    3000);
            } else {
                QMessageBox::warning(this, "Mission Switch Failed",
                                     QString::fromStdString(response->message));
            }
        });
}

void RovMonitorGui::toggleWaypointController() {
    bool runController = (runControllerButton_->text() == "Start Controller");
    auto request = std::make_shared<mundus_mir_msgs::srv::RunWaypointController::Request>();
    request->run = runController;
    
    auto result_future = runWaypointControllerClient_->async_send_request(
        request,
        [this, runController](rclcpp::Client<mundus_mir_msgs::srv::RunWaypointController>::SharedFuture future) {
            auto response = future.get();
            if (response->accepted) {
                runControllerButton_->setText(runController ? "Stop Controller" : "Start Controller");
                statusBar()->showMessage(QString("Waypoint controller %1").arg(runController ? "started" : "stopped"), 3000);
            } else {
                QMessageBox::warning(this, "Controller Toggle Failed", "Failed to change controller state");
            }
        });
}

void RovMonitorGui::clearWaypoints() {
    auto request = std::make_shared<mundus_mir_msgs::srv::ClearWaypoints::Request>();
    request->clear = true;
    
    auto result_future = clearWaypointsClient_->async_send_request(
        request,
        [this](rclcpp::Client<mundus_mir_msgs::srv::ClearWaypoints>::SharedFuture future) {
            auto response = future.get();
            if (response->accepted) {
                statusBar()->showMessage("Waypoints cleared", 3000);
            } else {
                QMessageBox::warning(this, "Clear Waypoints Failed", "Failed to clear waypoints");
            }
        });
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("rov_monitor_gui");
    
    QApplication app(argc, argv);
    RovMonitorGui gui(node);
    gui.show();
    
    std::thread rosSpinThread([&node]() {
        rclcpp::spin(node);
    });
    
    int result = app.exec();
    
    rclcpp::shutdown();
    if (rosSpinThread.joinable()) {
        rosSpinThread.join();
    }
    
    return result;
}

#include "rov_monitor_gui.moc"
