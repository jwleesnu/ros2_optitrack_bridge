#include "optitrack_bridge2/optitrack_bridge_pose_node.h"
#include "optitrack_bridge2/natnet_wrapper.h"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace optitrack {

OptitrackBridgePoseNode::OptitrackBridgePoseNode() : rclcpp::Node("optitrack_pose") {

    // parameter setting
    this->declare_parameter<std::string>("pose_prefix", "optitrack");
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<double>("hz", 100);
    this->get_parameter("pose_prefix", pose_prefix_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("hz", hz_);

    timer_ = this->create_wall_timer(
        std::chrono::operator""s(1.0 / hz_),
        std::bind(&OptitrackBridgePoseNode::loop_, this)
    );

    natnet_wrapper::NatNetWrapper::set_logger(this->get_logger());
    natnet_wrapper::NatNetWrapper::set_frame_id(frame_id_);

    
}

void OptitrackBridgePoseNode::loop_() {
    static bool initialized = false;
    if(!initialized) {
        if(natnet_wrapper::NatNetWrapper::run() != 0) {
            // if natnet wrapper initialization failed, escape after 100 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::shutdown();
            return;
        }
        initialized = true;

        // pause for 300 ms to prevent the node from publishing 0 0 0 ...
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    natnet_wrapper::NatNetWrapper::get_poses(body_names_, poses_);
    if(body_names_.size() != poses_.size()) {
        RCLCPP_WARN(this->get_logger(), "sizes of body_names(%d) and poses(%d) do not match!", static_cast<int>(body_names_.size()), static_cast<int>(poses_.size()));
        RCLCPP_WARN(this->get_logger(), "not publishing..");
        return;
    }

    for(int i = 0; i < body_names_.size(); ++i) {
        auto find_result = publishers_.find(body_names_[i]);
        if(find_result == publishers_.end()) {
            // add new topic
            std::string topic_name;
            if(pose_prefix_.length() > 0) {
                topic_name = pose_prefix_ + body_names_[i];
            }
            else {
                topic_name = body_names_[i];
            }
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
            publishers_.insert(std::make_pair(
                body_names_[i], pub
            ));
            pub->publish(poses_[i]);
        }
        else {
            // use existing publisher
            find_result->second->publish(poses_[i]);
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 3000, "%s is being published...", body_names_[i].c_str());
    }
}

}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<optitrack::OptitrackBridgePoseNode>());
    rclcpp::shutdown();
    return 0;
}
