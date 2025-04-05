// linear_kalman_filter.cpp (ROS2-compatible refactor)

#include "optitrack_bridge2/linear_kalman_filter.h"

LinearKalmanFilter::LinearKalmanFilter() {
    F = FMatrix::Zero();
    G = GMatrix::Zero();
    Q = QMatrix::Zero();
    H = HMatrix::Zero();

    x_old = NVector::Zero();
    x_predict = NVector::Zero();
    x_estimate = NVector::Zero();
    P_old = PMatrix::Zero();
    P_predict = PMatrix::Zero();
    P_estimate = PMatrix::Zero();

    sigma_Q = MVector::Constant(20.0);
    sigma_R = MVector::Constant(0.001);

    H.setZero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    H(2, 2) = 1.0;
}

nav_msgs::msg::Odometry LinearKalmanFilter::pose_cb(const geometry_msgs::msg::PoseStamped &msg) {
    nav_msgs::msg::Odometry odom;   

    if (initialized) {
        rclcpp::Time t_new = rclcpp::Time(msg.header.stamp);
        double dt = (t_new - t_old).seconds();

        predict(dt);
        update(dt, msg);

        odom.header = msg.header;
        odom.pose.pose = msg.pose;

        odom.twist.twist.linear.x = x_estimate(3, 0);
        odom.twist.twist.linear.y = x_estimate(4, 0);
        odom.twist.twist.linear.z = x_estimate(5, 0);

        // Estimate angular velocity from orientation difference (optional, basic version)
        double q0 = msg.pose.orientation.w;
        double q1 = msg.pose.orientation.x;
        double q2 = msg.pose.orientation.y;
        double q3 = msg.pose.orientation.z;

        double phi = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
        double theta = asin(2*(q0*q2 - q3*q1));
        double psi = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

        static double last_phi = phi, last_theta = theta, last_psi = psi;
        static rclcpp::Time last_time = msg.header.stamp;


        double dt_ang = (rclcpp::Time(msg.header.stamp) - last_time).seconds();
        if (dt_ang > 0.0) {
            odom.twist.twist.angular.x = (phi - last_phi) / dt_ang;
            odom.twist.twist.angular.y = (theta - last_theta) / dt_ang;
            odom.twist.twist.angular.z = (psi - last_psi) / dt_ang;

            last_phi = phi;
            last_theta = theta;
            last_psi = psi;
            last_time = rclcpp::Time(msg.header.stamp);
        }

        pose_old = msg;
        x_old = x_estimate;
        P_old = P_estimate;
        t_old = t_new;
    } else {
        t_old = msg.header.stamp;

        x_old(0, 0) = msg.pose.position.x;
        x_old(1, 0) = msg.pose.position.y;
        x_old(2, 0) = msg.pose.position.z;

        P_old.setIdentity();
        P_old.block<3,3>(0,0) *= 0.1;
        P_old.block<3,3>(3,3) *= 1.1;

        initialized = true;
    }

    return odom;
}

void LinearKalmanFilter::predict(const double &dt) {
    F = computeF(dt);
    G = computeG(dt);
    Q = computeQ(G, sigma_Q);

    x_predict = F * x_old;
    P_predict = F * P_old * F.transpose() + Q;
}

void LinearKalmanFilter::update(const double &dt, const geometry_msgs::msg::PoseStamped &msg) {
    MVector measure;
    measure << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;

    MVector residual = measure - H * x_predict;
    RMatrix R = computeR();
    RMatrix innovation = R + H * P_predict * H.transpose();
    GMatrix K = P_predict * H.transpose() * innovation.inverse();

    x_estimate = x_predict + K * residual;
    P_estimate = P_predict - K * innovation * K.transpose();
}

FMatrix LinearKalmanFilter::computeF(const double &dt) {
    FMatrix F = FMatrix::Identity();
    F(0, 3) = dt;
    F(1, 4) = dt;
    F(2, 5) = dt;
    return F;
}

GMatrix LinearKalmanFilter::computeG(const double &dt) {
    GMatrix G = GMatrix::Zero();
    for (int i = 0; i < 3; ++i) {
        G(i, i) = 0.5 * dt * dt;
        G(i + 3, i) = dt;
    }
    return G;
}

QMatrix LinearKalmanFilter::computeQ(const GMatrix &G, const MVector &sigma_Q) {
    RMatrix Qd = RMatrix::Zero();
    for (int i = 0; i < Col; ++i) {
        Qd(i, i) = sigma_Q(i, 0) * sigma_Q(i, 0);
    }
    return G * Qd * G.transpose();
}

RMatrix LinearKalmanFilter::computeR() {
    RMatrix R = RMatrix::Zero();
    for (int i = 0; i < Col; ++i) {
        R(i, i) = sigma_R(i, 0);
    }
    return R;
}

Eigen::Vector3d LinearKalmanFilter::getVelocity() const {
    return x_estimate.block<3,1>(3,0);
}

Eigen::Vector3d LinearKalmanFilter::getAngularVelocity() const {
    // optional: already embedded in pose_cb as differential
    return Eigen::Vector3d(0, 0, 0);
}
