#ifndef OPTITRACK_BRIDGE_LINEARKALMANFILTER_H
#define OPTITRACK_BRIDGE_LINEARKALMANFILTER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <string>

#define PUBLISH_RAW_TWIST false

#define Row 6
#define Col 3

using namespace Eigen;
typedef Matrix<double, Row, Row> FMatrix;
typedef Matrix<double, Row, Col> GMatrix;
typedef Matrix<double, Row, Row> QMatrix;
typedef Matrix<double, Row, Row> PMatrix;
typedef Matrix<double, Col, Col> RMatrix;
typedef Matrix<double, Col, Row> HMatrix;
typedef Matrix<double, Row, 1> NVector;
typedef Matrix<double, Col, 1> MVector;

class LinearKalmanFilter {
public:
    LinearKalmanFilter();
    nav_msgs::msg::Odometry pose_cb(const geometry_msgs::msg::PoseStamped& msg);

    Eigen::Vector3d getVelocity() const;
    Eigen::Vector3d getAngularVelocity() const;

private:
    FMatrix F;
    GMatrix G;
    QMatrix Q;
//    RMatrix R;
    HMatrix H;

    NVector x_old;
    NVector x_predict;
    NVector x_estimate;
    PMatrix P_old;
    PMatrix P_predict;
    PMatrix P_estimate;

    MVector sigma_Q;
    MVector sigma_R;

    geometry_msgs::msg::PoseStamped pose_old, pose_new;
    geometry_msgs::msg::TwistStamped twist, twist_raw;
    nav_msgs::msg::Odometry odom;

    rclcpp::Time t_old, t_new;
    bool initialized = false;

    void predict(const double &dt);
    static FMatrix computeF(const double &dt);
    static GMatrix computeG(const double &dt);
    static QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
    void update(const double &dt, const geometry_msgs::msg::PoseStamped& msg);
    RMatrix computeR();
};


#endif //OPTITRACK_BRIDGE_LINEARKALMANFILTER_H