#include "optitrack_bridge2/optitrack_bridge_odometry_node.h"
#include "optitrack_bridge2/natnet_wrapper.h"
#include "optitrack_bridge2/linear_kalman_filter.h"
#include <rclcpp/rclcpp.hpp>


using namespace std::chrono_literals;

namespace optitrack {

OptitrackBridgeOdometryNode::OptitrackBridgeOdometryNode() : rclcpp::Node("optitrack_odometry") {

    this->declare_parameter<std::string>("pose_prefix", "optitrack");
    this->declare_parameter<std::string>("frame_id", "world");
    this->declare_parameter<double>("hz", 100.0);

    this->get_parameter("pose_prefix", pose_prefix_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("hz", hz_);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / hz_),
        std::bind(&OptitrackBridgeOdometryNode::loop_, this)
    );

    natnet_wrapper::NatNetWrapper::set_logger(this->get_logger());
    natnet_wrapper::NatNetWrapper::set_frame_id(frame_id_);
}

void OptitrackBridgeOdometryNode::loop_() {
    static bool initialized = false;
    if (!initialized) {
        if (natnet_wrapper::NatNetWrapper::run() != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rclcpp::shutdown();
            return;
        }
        initialized = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }

    natnet_wrapper::NatNetWrapper::get_poses(body_names_, poses_);
    if (body_names_.size() != poses_.size()) {
        RCLCPP_WARN(this->get_logger(), "sizes of body_names(%d) and poses(%d) do not match!", static_cast<int>(body_names_.size()), static_cast<int>(poses_.size()));
        return;
    }

    for (size_t i = 0; i < body_names_.size(); ++i) {
        const std::string &name = body_names_[i];
        const auto &pose_msg = poses_[i];

        if (kalman_filters_.find(name) == kalman_filters_.end()) {
            kalman_filters_[name] = std::make_shared<LinearKalmanFilter>();
        }
        auto &filter = kalman_filters_[name];
        filter->pose_cb(pose_msg);

        Eigen::Vector3d lin_vel = filter->getVelocity();

        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = name;
        odom_msg.pose.pose = pose_msg.pose;
        odom_msg.twist.twist.linear.x = lin_vel.x();
        odom_msg.twist.twist.linear.y = lin_vel.y();
        odom_msg.twist.twist.linear.z = lin_vel.z();

        if (odom_publishers_.find(name) == odom_publishers_.end()) {
            std::string topic_name = pose_prefix_ + name;
            odom_publishers_[name] = this->create_publisher<nav_msgs::msg::Odometry>(topic_name, 1);
        }
        odom_publishers_[name]->publish(odom_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "%s odometry is being published...", name.c_str());
    }
}

} // namespace optitrack

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<optitrack::OptitrackBridgeOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
