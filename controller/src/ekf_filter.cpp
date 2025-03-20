#include<rclcpp/rclcpp.hpp>
#include<px4_msgs/msg/sensor_combined.hpp>
#include<px4_msgs/msg/vehicle_odometry.hpp>
#include<sensor_msgs/msg/imu.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<Eigen/Dense>
#include<chrono>

using std::placeholders::_1;
using namespace std::chrono_literals;

class EKFAltitude : public rclcpp::Node {
public:
    EKFAltitude(): Node("ekf_altitude_filter") , qos_profile(rclcpp::QoS(rclcpp::KeepLast(5))) {
        // Initialize matrices here
        x.setZero();
        P.setIdentity();
        F.setIdentity();
        H.setIdentity();
        Q = Eigen::MatrixXd::Identity(12, 12) * 0.1;
        R = Eigen::MatrixXd::Identity(12, 12) * 0.5;
        
        qos_profile = qos_profile.best_effort().transient_local();
        
        imu_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
            "/fmu/out/sensor_combined", qos_profile,
            std::bind(&EKFAltitude::imu_callback, this, _1));
            
        odom_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos_profile,
            std::bind(&EKFAltitude::odom_callback, this, _1));
            
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/ekf_pose", qos_profile);
    }

private:
    rclcpp::QoS qos_profile;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr imu_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    
    Eigen::Matrix<double, 12, 1> x;
    Eigen::Matrix<double, 12, 12> P, F, H, Q, R;
    const double dt = 0.02;
    const double gravity = 9.81;

    // Function declarations (only declare them once)
    Eigen::Quaterniond eular_to_quaternion(double roll, double pitch, double yaw);
    void imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg);
    void odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
};

// Implementations outside the class definition
void EKFAltitude::imu_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg) {
    Eigen::Vector3d raw_acc(msg->accelerometer_m_s2[0],
                            msg->accelerometer_m_s2[1],
                            msg->accelerometer_m_s2[2]);
    Eigen::Vector3d raw_gyro(msg->gyro_rad[0],
                             msg->gyro_rad[1],
                             msg->gyro_rad[2]);
    x.segment<3>(0) += x.segment<3>(3) * dt;
    x.segment<3>(3) += (raw_acc - Eigen::Vector3d(0, 0, gravity)) * dt;
    x.segment<3>(6) += raw_gyro * dt;
    P = F * P * F.transpose() + Q;
    
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = x(0);
    pose_msg.position.y = x(1);
    pose_msg.position.z = x(2);

    Eigen::Quaterniond q = eular_to_quaternion(x(6), x(7), x(8));
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();

    pose_pub_->publish(pose_msg);
}

void EKFAltitude::odom_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
    Eigen::Vector3d measurement(msg->velocity[0],
                                msg->velocity[1],
                                msg->velocity[2]);
    Eigen::Vector3d y = measurement - x.segment<3>(3);
    Eigen::Matrix3d S = P.block<3, 3>(3, 3) + R.block<3, 3>(3, 3);
    Eigen::Matrix3d K = P.block<3, 3>(3, 3) * S.inverse();
    x.segment<3>(3) += K * y;
    P.block<3, 3>(3, 3) -= K * P.block<3, 3>(3, 3);
}

Eigen::Quaterniond EKFAltitude::eular_to_quaternion(double roll, double pitch, double yaw) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) * 
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EKFAltitude>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}