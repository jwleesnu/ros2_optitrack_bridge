#include "optitrack_bridge2/optitrack_bridge_node.h"
#include "optitrack_bridge2/natnet_wrapper.h"

using namespace std::chrono_literals;

namespace optitrack {

OptitrackBridgeNode::OptitrackBridgeNode() : rclcpp::Node("optitrack_bridge2") {

    // parameter setting
    this->declare_parameter<std::string>("pose_prefix", "optitrack");
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<double>("hz", 100);
    this->get_parameter("pose_prefix", pose_prefix_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("hz", hz_);

    timer_ = this->create_wall_timer(
        std::chrono::operator""s(1.0 / hz_),
        std::bind(&OptitrackBridgeNode::loop_, this)
    );

    natnet_wrapper::NatNetWrapper::set_logger(this->get_logger());
    natnet_wrapper::NatNetWrapper::set_frame_id(frame_id_);
}

void OptitrackBridgeNode::loop_() {
    static bool initialized = false;
    if(!initialized) {
        if(natnet_wrapper::NatNetWrapper::run() != 0) {
            // if natnet wrapper initialization failed, escape
            rclcpp::shutdown();
            return;
        }
        initialized = true;
    }

    natnet_wrapper::NatNetWrapper::get_poses(body_names_, poses_);
    if(body_names_.size() != poses_.size()) {
        RCLCPP_WARN(this->get_logger(), "sizes of body_names(%d) and poses(%d) do not match!", body_names_.size(), poses_.size());
        RCLCPP_WARN(this->get_logger(), "not publishing..");
        return;
    }

    for(int i = 0; i < body_names_.size(); ++i) {
        auto find_result = publishers_.find(body_names_[i]);
        if(find_result == publishers_.end()) {
            // add new topic
            std::string topic_name;
            if(pose_prefix_.length() > 0) {
                topic_name = pose_prefix_ + "/" + body_names_[i];
            }
            else {
                topic_name = body_names_[i];
            }
            auto pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
            publishers_.insert(std::make_pair(
                body_names_[i], pub
            ));
            pub->publish(poses_[i]);
        }
        else {
            // use existing publisher
            find_result->second->publish(poses_[i]);
        }
    }
}

}
