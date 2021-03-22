#ifndef OPTITRACK_BRIDGE_NATNET_WRAPPER_H
#define OPTITRACK_BRIDGE_NATNET_WRAPPER_H

#include <vector>
#include <string>
#include <memory>
#include <atomic>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "natnet/NatNetTypes.h"
#include "natnet/NatNetCAPI.h"
#include "natnet/NatNetClient.h"

namespace natnet_wrapper {

// thread-safe singleton wrapper class (since C++11)
class NatNetWrapper {
    public:
    // get instance (singleton)
    static NatNetWrapper& instance();

    static void server_discovered_callback(const sNatNetDiscoveredServer* p_discovered_server, void* p_user_context);
    static void data_handler(sFrameOfMocapData* data, void* p_user_data);
    static void message_handler(Verbosity msg_type, const char* msg);
    static void reset_client();
    static int connect_client();

    static void set_verbosity(const Verbosity& verbosity);
    static void set_logger(const rclcpp::Logger& logger);
    static void set_frame_id(const std::string& frame_id);

    static int run();

    static void get_poses(std::vector<std::string>& body_names, std::vector<geometry_msgs::msg::PoseStamped>& poses);

    ~NatNetWrapper();

    private:
    NatNetWrapper();
    // NatNetWrapper(const NatNetWrapper&);
    // NatNetWrapper& operator=(const NatNetWrapper&);

    Verbosity verbosity_;
    const bool show_latency_;
    std::vector<int> model_ids_;
    const ConnectionType default_connection_type_;
    sNatNetClientConnectParams connect_params_;
    std::unique_ptr<NatNetClient> client_;
    std::vector<sNatNetDiscoveredServer> discovered_servers_;
    char discovered_multicast_group_addr_[kNatNetIpv4AddrStrLenMax];
    int analog_samples_per_mocap_frame_;
    sServerDescription server_description_;
    std::atomic<bool> is_server_discovered_;

    // is client running
    std::atomic<bool> listening_;

    // rclcpp msg
    std::mutex poses_mutex_;
    std::vector<geometry_msgs::msg::PoseStamped> poses_;
    std::vector<std::string> body_names_;
    std::string frame_id_;

    // rclcpp logger
    rclcpp::Logger logger_;

    // rclcpp time (defaults to system time)
    rclcpp::Clock clock_;

    public:
    NatNetWrapper(const NatNetWrapper&) = delete;
    NatNetWrapper& operator=(const NatNetWrapper&) = delete;

};

}

#endif
