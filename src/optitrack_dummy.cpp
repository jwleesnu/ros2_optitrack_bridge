#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <limits>

using namespace std::chrono_literals;

namespace optitrack {

class DummyVisualOdomPublisher : public rclcpp::Node {
public:
  DummyVisualOdomPublisher()
    : Node("dummy_visual_odom_publisher")
  {
    // 파라미터 설정 (기본값: hz = 100)
    this->declare_parameter<double>("hz", 100);
    this->get_parameter("hz", hz_);

    // VehicleOdometry 메시지를 publish할 publisher 생성 (단일 토픽)
    publisher_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 1);

    // 타이머 생성: hz 값에 따라 주기적으로 loop() 호출
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&DummyVisualOdomPublisher::loop, this)
    );
  }

private:
  void loop() {
    // 현재 시간을 이용하여 dummy PoseStamped 시간 정보를 대체
    auto current_time = this->now();

    // VehicleOdometry 메시지 생성 및 채우기
    px4_msgs::msg::VehicleOdometry odom_msg;
    odom_msg.timestamp = current_time.nanoseconds() / 1000;
    odom_msg.timestamp_sample = current_time.nanoseconds() / 1000;

    // 좌표계: 예시로 NED 사용 (pose_frame = 1)
    odom_msg.pose_frame = 1;
    // dummy 위치 (ROS의 NWU 좌표계를 가정하여 NED로 변환)
    odom_msg.position[0] = 1.0;        // north
    odom_msg.position[1] = -2.0;       // west -> east: 부호 반전
    odom_msg.position[2] = -3.0;       // up -> down: 부호 반전

    // dummy orientation (identity quaternion)
    odom_msg.q[0] = 0.0;
    odom_msg.q[1] = 0.0;
    odom_msg.q[2] = 0.0;
    odom_msg.q[3] = 1.0;

    // 속도 관련 필드 (dummy 값: NaN)
    odom_msg.velocity_frame = 1;
    float nan_val = std::numeric_limits<float>::quiet_NaN();
    for (int i = 0; i < 3; ++i) {
      odom_msg.velocity[i] = nan_val;
      odom_msg.angular_velocity[i] = nan_val;
      odom_msg.position_variance[i] = nan_val;
      odom_msg.orientation_variance[i] = nan_val;
      odom_msg.velocity_variance[i] = nan_val;
    }
    odom_msg.reset_counter = 0;
    odom_msg.quality = 0;

    // 메시지 publish
    publisher_->publish(odom_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Dummy VehicleOdometry published with timestamp: %lu", odom_msg.timestamp);
  }

  double hz_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace optitrack

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<optitrack::DummyVisualOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}