#include "blueye_bt_real/bt_executor_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    
    // Create and spin the BT executor node
    auto node = std::make_shared<blueye_bt_real::BTExecutorNode>();
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}