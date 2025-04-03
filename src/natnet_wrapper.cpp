#include <thread>
#include <chrono>
#include <inttypes.h>

#include <rclcpp/time.hpp>

#include "optitrack_bridge2/natnet_wrapper.h"

// void server_discovered_callback_wrapper(const sNatNetDiscoveredServer* p_discovered_server, void* p_user_context) {
//     natnet_wrapper::NatNetWrapper::server_discovered_callback(p_discovered_server, p_user_context);
// }

namespace natnet_wrapper {

NatNetWrapper& NatNetWrapper::instance() {
    static NatNetWrapper* i = new NatNetWrapper();
    return *i;
}

void NatNetWrapper::server_discovered_callback(const sNatNetDiscoveredServer* p_discovered_server, void* p_user_context) {
    char server_hotkey = '.';
    if(instance().discovered_servers_.size() < 9) {
        server_hotkey = static_cast<char>('1' + instance().discovered_servers_.size());
    }

    const char* warning = "";
    bool have_warning = false;
    if(!(p_discovered_server->serverDescription.bConnectionInfoValid)) {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
        have_warning = true;
    }

    if(have_warning) {
        RCLCPP_WARN(instance().logger_,
            "[%c] %s %d.%d at %s%s",
            server_hotkey,
            p_discovered_server->serverDescription.szHostApp,
            p_discovered_server->serverDescription.HostAppVersion[0],
            p_discovered_server->serverDescription.HostAppVersion[1],
            p_discovered_server->serverAddress,
            warning
        );
    }
    else {
        RCLCPP_INFO(instance().logger_,
            "[%c] %s %d.%d at %s",
            server_hotkey,
            p_discovered_server->serverDescription.szHostApp,
            p_discovered_server->serverDescription.HostAppVersion[0],
            p_discovered_server->serverDescription.HostAppVersion[1],
            p_discovered_server->serverAddress
        );
    }

    instance().discovered_servers_.push_back(*p_discovered_server);
    instance().is_server_discovered_ = true;
}

void NatNetWrapper::data_handler(sFrameOfMocapData* data, void* p_user_data) {
    NatNetClient* client = (NatNetClient*) p_user_data;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softward_latency_host_ticks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double software_latency_ms = client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transit_latency_ms = client->SecondsSinceHostTimestamp(data->TransmitTimestamp) * 1000.0;

    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool system_latency_available = data->CameraMidExposureTimestamp != 0;

    if(instance().show_latency_) {
        if(system_latency_available) {
            // System latency here is defined as the span of time between:
            //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
            // and
            //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
            const uint64_t system_latency_host_ticks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
            const double system_latency_ms = (system_latency_host_ticks * 1000) / static_cast<double>(instance().server_description_.HighResClockFrequency);

            // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
            // This is the all-inclusive measurement (photons to client processing).
            const double client_latency_ms = client->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

            // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
            //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

            RCLCPP_INFO(instance().logger_, "Software latency : %.2lf milliseconds", software_latency_ms);
            RCLCPP_INFO(instance().logger_, "System latency : %.2lf milliseconds", system_latency_ms);
            RCLCPP_INFO(instance().logger_, "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)", client_latency_ms, transit_latency_ms);
        }
        else {
            RCLCPP_INFO(instance().logger_, "Transit latency : %.2lf milliseconds", transit_latency_ms);
        }
    }

    // FrameOfMocapData params
    bool is_recording = (data->params & 0x01) != 0;
    bool tracked_models_changed = (data->params & 0x02) != 0;
    if(is_recording) {
        RCLCPP_INFO(instance().logger_, "RECORDING");
    }
    if(tracked_models_changed) {
        RCLCPP_INFO(instance().logger_, "Models changed.");
    }

    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
    // decode to friendly string
    char time_code_cstr[128] = "";
    NatNet_TimecodeStringify(data->Timecode, data->TimecodeSubframe, time_code_cstr, 128);

    // rigid bodies
    for(int i = 0; i < data->nRigidBodies; ++i) {
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool tracking_valid = data->RigidBodies[i].params & 0x01;

        if(tracking_valid && (instance().poses_.size() == data->nRigidBodies)) {
            const std::lock_guard<std::mutex> poses_lock(instance().poses_mutex_);
            for(int i = 0; i < data->nRigidBodies; ++i) {
                instance().poses_[i].header.stamp = instance().clock_.now();
                instance().poses_[i].header.frame_id = instance().frame_id_;
                instance().poses_[i].pose.position.x = data->RigidBodies[i].x;
                instance().poses_[i].pose.position.y = data->RigidBodies[i].y;
                instance().poses_[i].pose.position.z = data->RigidBodies[i].z;
                instance().poses_[i].pose.orientation.x = data->RigidBodies[i].qx;
                instance().poses_[i].pose.orientation.y = data->RigidBodies[i].qy;
                instance().poses_[i].pose.orientation.z = data->RigidBodies[i].qz;
                instance().poses_[i].pose.orientation.w = data->RigidBodies[i].qw;
            }
        }
        else {
            RCLCPP_WARN_THROTTLE(
                instance().logger_, instance().clock_, 500,
                "%s pose tracking invalid", instance().body_names_[i].c_str()
            );
        }
    }
}

void NatNetWrapper::message_handler(Verbosity msg_type, const char* msg) {
    if(msg_type >= instance().verbosity_) {
        switch(msg_type) {
            case Verbosity_None:
            case Verbosity_Debug:
            RCLCPP_DEBUG(instance().logger_, "%s", msg);
            break;

            case Verbosity_Info:
            RCLCPP_INFO(instance().logger_, "%s", msg);
            break;

            case Verbosity_Warning:
            RCLCPP_WARN(instance().logger_, "%s", msg);
            break;

            case Verbosity_Error:
            RCLCPP_ERROR(instance().logger_, "%s", msg);
            break;

            default:
            break;
        }
    }
}

void NatNetWrapper::reset_client() {
    int success;
    RCLCPP_INFO(instance().logger_, "resetting client..");

    success = instance().client_->Disconnect();
    if(success != 0) {
        RCLCPP_ERROR(instance().logger_, "error disconnecting client");
    }

    success = instance().client_->Connect(instance().connect_params_);
    if(success != 0) {
        RCLCPP_ERROR(instance().logger_, "error reconnecting client");
    }
}

int NatNetWrapper::connect_client() {
    instance().client_->Disconnect();
    
    int ret_code = instance().client_->Connect(instance().connect_params_);
    if(ret_code != ErrorCode_OK) {
        RCLCPP_ERROR(instance().logger_, "Unable to connect to server. Error code : %d. Exiting..", ret_code);
        return ErrorCode_Internal;
    }

    // connection succeeded
    void* result;
    int n_bytes = 0;
    ErrorCode ret = ErrorCode_OK;

    // print server info
    memset(&(instance().server_description_), 0, sizeof(instance().server_description_));
    ret = instance().client_->GetServerDescription(&(instance().server_description_));
    if(ret != ErrorCode_OK || !instance().server_description_.HostPresent) {
        RCLCPP_ERROR(instance().logger_, "Unable to connect to server. Host not present. Exiting..");
    }
    RCLCPP_INFO(instance().logger_, "Server application info: ");
    RCLCPP_INFO(instance().logger_,
        "Application: %s (ver. %d.%d.%d.%d)",
        instance().server_description_.szHostApp,
        instance().server_description_.HostAppVersion[0], instance().server_description_.HostAppVersion[1],
        instance().server_description_.HostAppVersion[2], instance().server_description_.HostAppVersion[3]
    );
    RCLCPP_INFO(instance().logger_, 
        "NatNet Version: %d.%d.%d.%d",
        instance().server_description_.NatNetVersion[0], instance().server_description_.NatNetVersion[1],
        instance().server_description_.NatNetVersion[2], instance().server_description_.NatNetVersion[3]
    );
    RCLCPP_INFO(instance().logger_, "Client IP:%s", instance().connect_params_.localAddress);
    RCLCPP_INFO(instance().logger_, "Server IP:%s", instance().connect_params_.serverAddress);
    RCLCPP_INFO(instance().logger_, "Server Name:%s", instance().server_description_.szHostComputerName);

    // get mocap frame rate
    ret = instance().client_->SendMessageAndWait("FrameRate", &result, &n_bytes);
    if(ret == ErrorCode_OK) {
        float rate = *((float*)result);
        RCLCPP_INFO(instance().logger_, "Mocap Frame Rate : %3.2f", rate);
    }
    else {
        RCLCPP_ERROR(instance().logger_, "Error getting frame rate.");
    }

    return ErrorCode_OK;
}

void NatNetWrapper::set_verbosity(const Verbosity& verbosity) {
    instance().verbosity_ = verbosity;
}

void NatNetWrapper::set_logger(const rclcpp::Logger& logger) {
    instance().logger_ = logger;
}

void NatNetWrapper::set_frame_id(const std::string& frame_id) {
    instance().frame_id_ = frame_id;
}

// constructor
NatNetWrapper::NatNetWrapper()
  : logger_(rclcpp::get_logger("NatNetWrapper")),
    default_connection_type_(ConnectionType_Multicast),
    client_(nullptr),
    is_server_discovered_(false),
    verbosity_(Verbosity_Debug),
    listening_(false),
    show_latency_(false)
{ }

int NatNetWrapper::run() {
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion(ver);
    RCLCPP_INFO(instance().logger_, "NatNet ver. %d.%d.%d.%d", ver[0], ver[1], ver[2], ver[3]);

    // Install logging callback
    NatNet_SetLogCallback(NatNetWrapper::message_handler);

    instance().client_ = std::make_unique<NatNetClient>();

    // set the frame callback handler
    instance().client_->SetFrameReceivedCallback(NatNetWrapper::data_handler, instance().client_.get());

    // attempt to discover servers on the local network.
    // do asynchronous server discovery
    RCLCPP_INFO(instance().logger_, "looking for servers on the local network");

    instance().is_server_discovered_ = false;
    NatNetDiscoveryHandle discovery;
    // NatNet_CreateAsyncServerDiscovery(&discovery, NatNetWrapper::server_discovered_callback);
    NatNet_CreateAsyncServerDiscovery(&discovery, NatNetWrapper::server_discovered_callback);

    // try for 2 second
    for(int i = 0; i < 200; ++i) {
        if(instance().is_server_discovered_) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    NatNet_FreeAsyncServerDiscovery( discovery );

    // if server is not discovered, leave.
    if(!instance().is_server_discovered_) {
        RCLCPP_ERROR(instance().logger_, "no server discovered. Exiting..");
        return 1;
    }

    // choose the first server
    const size_t server_index = 0;
    const sNatNetDiscoveredServer& discovered_server = instance().discovered_servers_[server_index];

    if(discovered_server.serverDescription.bConnectionInfoValid) {
        // Build the connection parameters.
        snprintf(
            instance().discovered_multicast_group_addr_,
            sizeof(instance().discovered_multicast_group_addr_),
            "%" PRIu8".%" PRIu8".%" PRIu8".%" PRIu8"",
            discovered_server.serverDescription.ConnectionMulticastAddress[0],
            discovered_server.serverDescription.ConnectionMulticastAddress[1],
            discovered_server.serverDescription.ConnectionMulticastAddress[2],
            discovered_server.serverDescription.ConnectionMulticastAddress[3]
        );

        instance().connect_params_.connectionType = discovered_server.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
        instance().connect_params_.serverCommandPort = discovered_server.serverCommandPort;
        instance().connect_params_.serverDataPort = discovered_server.serverDescription.ConnectionDataPort;
        instance().connect_params_.serverAddress = discovered_server.serverAddress;
        instance().connect_params_.localAddress = discovered_server.localAddress;
        instance().connect_params_.multicastAddress = instance().discovered_multicast_group_addr_;
    }
    else {
        // We're missing some info because it's a legacy server.
        // Guess the defaults and make a best effort attempt to connect.
        instance().connect_params_.connectionType = instance().default_connection_type_;
        instance().connect_params_.serverCommandPort = discovered_server.serverCommandPort;
        instance().connect_params_.serverDataPort = 0;
        instance().connect_params_.serverAddress = discovered_server.serverAddress;
        instance().connect_params_.localAddress = discovered_server.localAddress;
        instance().connect_params_.multicastAddress = NULL;
    }

    // connect to Motive
    int result;
    result = connect_client();
    if(result != ErrorCode_OK) {
        RCLCPP_ERROR(instance().logger_, "Error initializing client. See log for details. Exiting..");
        return 1;
    }
    else {
        RCLCPP_INFO(instance().logger_, "Client initialized and ready.");
    }

    // send/receive test request
    void* response;
    int n_bytes;
    RCLCPP_INFO(instance().logger_, "Sending test request..");
    result = instance().client_->SendMessageAndWait("TestRequest", &response, &n_bytes);
    if(result == ErrorCode_OK) {
        RCLCPP_INFO(instance().logger_, "Received: %s", (char*)response);
    }
    else {
        RCLCPP_WARN(instance().logger_, "Received: %s", (char*)response);
    }

    RCLCPP_INFO(instance().logger_, "Requesting data descriptions..");
    sDataDescriptions* data_defs = NULL;
    result = instance().client_->GetDataDescriptionList(&data_defs);
    if(result != ErrorCode_OK || data_defs == NULL) {
        RCLCPP_WARN(instance().logger_, "Unable to retrieve data descriptions.");
        return 1;
    }
    else {
        RCLCPP_INFO(instance().logger_, "Received %d data descriptions: ", data_defs->nDataDescriptions);
        for(int i = 0; i < data_defs->nDataDescriptions; ++i) {
            RCLCPP_INFO(instance().logger_, "Data description # %d (type=%d)", i, data_defs->arrDataDescriptions[i].type);

            // add additional data description type here for more logging
            switch(data_defs->arrDataDescriptions[i].type) {
                case Descriptor_MarkerSet:
                {
                    sMarkerSetDescription* ms = data_defs->arrDataDescriptions[i].Data.MarkerSetDescription;
                    RCLCPP_INFO(instance().logger_, "Markerset name : %s", ms->szName);
                    for(int j = 0; j < ms->nMarkers; ++j) {
                        RCLCPP_INFO(instance().logger_, "%s", ms->szMarkerNames[j]);
                    }
                }
                break;

                case Descriptor_RigidBody:
                {
                    const std::lock_guard<std::mutex> poses_lock(instance().poses_mutex_);
                    sRigidBodyDescription* rb = data_defs->arrDataDescriptions[i].Data.RigidBodyDescription;
                    RCLCPP_INFO(instance().logger_, "Rigid body name : %s", rb->szName);
                    RCLCPP_INFO(instance().logger_, "Rigid body ID : %d", rb->ID);
                    RCLCPP_INFO(instance().logger_, "Rigid body parent ID : %d", rb->parentID);
                    RCLCPP_INFO(instance().logger_, "Parent offset : %3.2f, %3.2f, %3.2f", rb->offsetx, rb->offsety, rb->offsetz);
                    instance().model_ids_.push_back(rb->ID);
                    geometry_msgs::msg::PoseStamped pose;
                    instance().poses_.push_back(pose);
                    instance().body_names_.push_back(std::string(rb->szName));
                }
                break;

                default:
                break;
            }
        }

        // ready to receive marker stream
        instance().listening_ = true;
        return 0;
    }
}

void NatNetWrapper::get_poses(std::vector<std::string>& body_names, std::vector<geometry_msgs::msg::PoseStamped>& poses) {
    const std::lock_guard<std::mutex> poses_lock(instance().poses_mutex_);
    body_names = instance().body_names_;
    poses = instance().poses_;
}

NatNetWrapper::~NatNetWrapper() {
    if(listening_) {
        // clean up
        client_->Disconnect();
    }
    delete &instance();
}

}
