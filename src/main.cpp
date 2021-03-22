#include <rclcpp/rclcpp.hpp>
#include "optitrack_bridge2/optitrack_bridge_node.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<optitrack::OptitrackBridgeNode>());
    rclcpp::shutdown();
    return 0;
}
