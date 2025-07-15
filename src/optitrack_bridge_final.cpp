#include "optitrack_bridge2/optitrack_bridge_final_node.h"
#include "optitrack_bridge2/natnet_wrapper.h"
#include "optitrack_bridge2/linear_kalman_filter.h"
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

using namespace std::chrono_literals;

namespace optitrack {

    OptitrackBridgeFinalNode::OptitrackBridgeFinalNode() : rclcpp::Node("optitrack_final") {

        // parameter setting
        this->declare_parameter<std::string>("pose_prefix", "optitrack");
        this->declare_parameter<std::string>("frame_id", "world");
        this->declare_parameter<double>("hz", 100);
        this->get_parameter("pose_prefix", pose_prefix_);
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("hz", hz_);

        timer_ = this->create_wall_timer(
            std::chrono::operator""s(1.0 / hz_),
            std::bind(&OptitrackBridgeFinalNode::loop_, this)
        );
        
        natnet_wrapper::NatNetWrapper::set_logger(this->get_logger());
        natnet_wrapper::NatNetWrapper::set_frame_id(frame_id_);
        
        publisher_vehicle_visual_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 1);

    }

    void OptitrackBridgeFinalNode::loop_() {

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
        if(body_names_.size() >= 2) {
            RCLCPP_WARN(this->get_logger(), "only one body can be linked to a px4");
            return;
        }
        const std::string &name = body_names_[0];
        const auto &pose_msg = poses_[0];
        
        px4_msgs::msg::VehicleOdometry visual_odom_msg;


        if (kalman_filters_.find(name) == kalman_filters_.end()) {
            kalman_filters_[name] = std::make_shared<LinearKalmanFilter>();
        }
        auto &filter = kalman_filters_[name];
        filter->pose_cb(pose_msg);

        Eigen::Vector3d lin_vel = filter->getVelocity();
        

        visual_odom_msg.timestamp = this->now().nanoseconds() / 1000;
        visual_odom_msg.timestamp_sample = (static_cast<uint64_t>(pose_msg.header.stamp.sec) * 1000000ULL)+(pose_msg.header.stamp.nanosec / 1000);

        visual_odom_msg.pose_frame = 1;

        visual_odom_msg.position[0] = pose_msg.pose.position.x;
        visual_odom_msg.position[1] = -pose_msg.pose.position.y;
        visual_odom_msg.position[2] = -pose_msg.pose.position.z;

        visual_odom_msg.q[0] = pose_msg.pose.orientation.w;
        visual_odom_msg.q[1] = pose_msg.pose.orientation.x;
        visual_odom_msg.q[2] = -pose_msg.pose.orientation.y;
        visual_odom_msg.q[3] = -pose_msg.pose.orientation.z;


        float nan_val = std::numeric_limits<float>::quiet_NaN();

        visual_odom_msg.velocity[0] = nan_val;
        visual_odom_msg.velocity[1] = nan_val;
        visual_odom_msg.velocity[2] = nan_val;

        visual_odom_msg.position_variance[0] = 0.0001;
        visual_odom_msg.position_variance[1] = 0.0001;
        visual_odom_msg.position_variance[2] = 0.0001;

        visual_odom_msg.orientation_variance[0] = 0.001;
        visual_odom_msg.orientation_variance[1] = 0.001;
        visual_odom_msg.orientation_variance[2] = 0.0005;

        visual_odom_msg.velocity_frame = 1;

        for (int i = 0; i < 3; ++i) {
            visual_odom_msg.angular_velocity[i] = nan_val;
            visual_odom_msg.velocity_variance[i] = nan_val;
        }

        publisher_vehicle_visual_odometry_->publish(visual_odom_msg);

        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *(this->get_clock()), 3000, "%s is being published...", body_names_[0].c_str());
    }

}//end namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<optitrack::OptitrackBridgeFinalNode>());
    rclcpp::shutdown();
    return 0;
}
