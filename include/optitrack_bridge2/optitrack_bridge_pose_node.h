#ifndef OPTITRACK_BRIDGE_NODE_H
#define OPTITRACK_BRIDGE_NODE_H

#include <vector>
#include <memory>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "optitrack_bridge2/natnet_wrapper.h"

namespace optitrack {

class OptitrackBridgePoseNode : public rclcpp::Node {

    public:
    OptitrackBridgePoseNode();

    private:
    rclcpp::TimerBase::SharedPtr timer_;

    // parameters
    std::string pose_prefix_;
    std::string frame_id_;
    double hz_;

    // poses
    std::vector<std::string> body_names_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;

    // publishers
    std::unordered_map<
        std::string,
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>>
    > publishers_;

    // loop callbacks
    void loop_();

};


}//end namespace

#endif
