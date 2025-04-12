// Header files
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/path.hpp>

#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class OffboardControlNode : public rclcpp::Node {
public:
    OffboardControlNode() : Node("offboard_control_cpp") {
        // Publishers
        rclcpp::QoS custom_sensor_qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        custom_sensor_qos
          .keep_last(10)
          .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
          .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE)
          .history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
                rclcpp::QoS qos_profile(rclcpp::QoS(10).best_effort().transient_local());
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);
        trajectory_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
        command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", qos_profile);

        // Subscribers
        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", custom_sensor_qos,
            std::bind(&OffboardControlNode::position_cb, this, _1));

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", custom_sensor_qos,
            std::bind(&OffboardControlNode::status_cb, this, _1));

        manual_goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/manual_setpoint", qos_profile, std::bind(&OffboardControlNode::manual_goal_cb, this, _1));

        // Services
        arm_srv_ = this->create_service<std_srvs::srv::SetBool>("/arm_drone", std::bind(&OffboardControlNode::arm_cb, this, _1, _2));
        land_srv_ = this->create_service<std_srvs::srv::Trigger>("/land_drone", std::bind(&OffboardControlNode::land_cb, this, _1, _2));

        // Timers
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControlNode::publish_control_loop, this));

        RCLCPP_INFO(this->get_logger(), "OffboardControlNode initialized");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr manual_goal_sub_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;

    rclcpp::TimerBase::SharedPtr timer_;

    px4_msgs::msg::VehicleLocalPosition current_position_;
    px4_msgs::msg::VehicleStatus vehicle_status_;

    std::vector<geometry_msgs::msg::PoseStamped> path_;
    size_t waypoint_index_ = 0;

    void position_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        current_position_ = *msg;
    }

    void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicle_status_ = *msg;
    }

    void manual_goal_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
        path_.clear();
        waypoint_index_ = 0;

        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.pose.position.x = current_position_.x + (msg->x - current_position_.x) * i / 4.0;
            pose.pose.position.y = current_position_.y + (msg->y - current_position_.y) * i / 4.0;
            pose.pose.position.z = current_position_.z + (msg->z - current_position_.z) * i / 4.0;
            path_.push_back(pose);
        }
        publish_path();
    }

    void publish_path() {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();
        path_msg.poses = path_;
        path_pub_->publish(path_msg);
    }

    void publish_control_loop() {
        publish_offboard_control_mode();

        if (waypoint_index_ < path_.size()) {
            const auto &pose = path_[waypoint_index_].pose.position;
            if (distance_to(pose) < 0.5) waypoint_index_++;
            publish_trajectory_setpoint(pose.x, pose.y, pose.z);
        }
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->now().nanoseconds() / 1000;
        offboard_control_mode_pub_->publish(msg);
    }

    void publish_trajectory_setpoint(float x, float y, float z) {
        px4_msgs::msg::TrajectorySetpoint sp;
        sp.position[0] = x;
        sp.position[1] = y;
        sp.position[2] = z;
        sp.yaw = 0.0;
        sp.timestamp = this->now().nanoseconds() / 1000;
        trajectory_pub_->publish(sp);
    }

    float distance_to(const geometry_msgs::msg::Point &p) {
        float dx = p.x - current_position_.x;
        float dy = p.y - current_position_.y;
        float dz = p.z - current_position_.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void arm_cb(const std_srvs::srv::SetBool::Request::SharedPtr req,
                const std_srvs::srv::SetBool::Response::SharedPtr res) {
        px4_msgs::msg::VehicleCommand cmd;
        cmd.command = 400;  // ARM/DISARM
        cmd.param1 = req->data ? 1.0 : 0.0;
        cmd.param2 = 21196.0;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->now().nanoseconds() / 1000;
        command_pub_->publish(cmd);

        res->success = true;
        res->message = req->data ? "Drone armed" : "Drone disarmed";
    }

    void land_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                 const std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        px4_msgs::msg::VehicleCommand cmd;
        cmd.command = 21; // LAND
        cmd.param1 = 0.0;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->now().nanoseconds() / 1000;
        command_pub_->publish(cmd);

        res->success = true;
        res->message = "Landing initiated";
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
