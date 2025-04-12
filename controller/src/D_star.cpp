#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <rclcpp/qos.hpp>

#include <armadillo>
#include <limits>
#include <functional>
#include <tuple>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <unordered_set>
#include <algorithm>

using std::placeholders::_1 ;
using std::placeholders::_2;
using namespace std::chrono_literals;
using Node3D = std::tuple<int, int, int>;

namespace std {
    template<>
    struct hash<Node3D> {
        std::size_t operator()(const Node3D &k) const {
            return std::get<0>(k) * 73856093 ^ std::get<1>(k) * 19349663 ^ std::get<2>(k) * 83492791;
        }
    };
}

class DStarLite3D {
public:
    DStarLite3D(const std::vector<int>& grid_size, const Node3D& start, const Node3D& goal)
        : grid_size_(grid_size), start_(start), goal_(goal), m_(0.0) {
        grid_.resize(grid_size[0], std::vector<std::vector<int>>(grid_size[1], std::vector<int>(grid_size[2], 1)));
        for (int x = 0; x < grid_size[0]; ++x)
            for (int y = 0; y < grid_size[1]; ++y)
                for (int z = 0; z < grid_size[2]; ++z) {
                    Node3D node = {x, y, z};
                    g_[node] = rhs_[node] = std::numeric_limits<double>::infinity();
                }
        rhs_[goal_] = 0.0;
        queue_.push(calculate_key(goal_));
    }
    void set_start(const Node3D& s){start_= s;}
    void set_goal(const Node3D& g) { goal_ = g; rhs_[goal_] = 0.0; queue_.push(calculate_key(goal_)); }

    
    
    void compute_shortest_path() {
        while (!queue_.empty()) {
            auto k_old = queue_.top(); queue_.pop();
            Node3D u = std::get<2>(k_old);
            if (g_[u] > rhs_[u]) {
                g_[u] = rhs_[u];
                for (auto &n : neighbors(u)) update_vertex(n);
            } else {
                g_[u] = std::numeric_limits<double>::infinity();
                for (auto &n : neighbors(u)) update_vertex(n);
                update_vertex(u);
            }
        }
    }
    std::vector<Node3D> get_path() {
        std::vector<Node3D> path = {start_};
        Node3D current = start_;
        while (current != goal_) {
            double min_cost = std::numeric_limits<double>::infinity();
            Node3D next = current;
            for (auto &n : neighbors(current)) {
                double c = g_[n] + cost(current, n);
                if (c < min_cost) {
                    min_cost = c;
                    next = n;
                }
            }
            if (next == current) break;
            path.push_back(next);
            current = next;
        }
        return path;
    }

    
    private:
    std::vector<int> grid_size_;
    Node3D start_, goal_;
    double m_;
    std::vector<std::vector<std::vector<int>>> grid_;
    std::unordered_map<Node3D, double> g_, rhs_;
    using Key = std::tuple<double, double, Node3D>;
    std::priority_queue<Key, std::vector<Key>, std::greater<>> queue_;

    double heuristic(const Node3D& a, const Node3D& b) {
        arma::vec va = {double(std::get<0>(a)), double(std::get<1>(a)), double(std::get<2>(a))};
        arma::vec vb = {double(std::get<0>(b)), double(std::get<1>(b)), double(std::get<2>(b))};
        return arma::norm(va - vb);
    }

    double cost(const Node3D& a, const Node3D& b) {
        if (grid_[std::get<0>(b)][std::get<1>(b)][std::get<2>(b)] == 0)
            return std::numeric_limits<double>::infinity();
        return heuristic(a, b);
    }

    Key calculate_key(const Node3D& node) {
        double min_g_rhs = std::min(g_[node], rhs_[node]);
        return {min_g_rhs + heuristic(start_, node) + m_, min_g_rhs, node};
    }

    void update_vertex(const Node3D& node) {
        if (node != goal_) {
            double min_rhs = std::numeric_limits<double>::infinity();
            for (auto& n : neighbors(node))
                min_rhs = std::min(min_rhs, g_[n] + cost(node, n));
            rhs_[node] = min_rhs;
        }
        queue_.push(calculate_key(node));
    }

    std::vector<Node3D> neighbors(const Node3D& node) {
        std::vector<Node3D> result;
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    int nx = std::get<0>(node) + dx;
                    int ny = std::get<1>(node) + dy;
                    int nz = std::get<2>(node) + dz;
                    if (nx >= 0 && ny >= 0 && nz >= 0 &&
                        nx < grid_size_[0] && ny < grid_size_[1] && nz < grid_size_[2])
                        result.emplace_back(nx, ny, nz);
                }
        return result;
    }
};

    

class DStarLiteNode : public rclcpp::Node {
    public:
        DStarLiteNode() : Node("dstar_lite") {
            planner_ = std::make_shared<DStarLite3D>(std::vector<int>{20, 20, 10}, Node3D{0,0,0}, Node3D{19,19,5});
    
            odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
                "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
                std::bind(&DStarLiteNode::odom_cb, this, _1));
    
            waypoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
            offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
            vehicle_cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);
            setpoint_sub_ = create_subscription<geometry_msgs::msg::Point>(
                "/manual_setpoint", 10, std::bind(&DStarLiteNode::manual_goal_cb, this, _1));
            path_pub_ = create_publisher<nav_msgs::msg::Path>("planned_path", 10);
    
            arm_service_ = create_service<std_srvs::srv::SetBool>(
                "arm_drone", std::bind(&DStarLiteNode::arm_cb, this, _1, _2));
            land_service_ = create_service<std_srvs::srv::Trigger>(
                "land_drone", std::bind(&DStarLiteNode::land_cb, this, _1, _2));
    
            offboard_timer_ = create_wall_timer(50ms, std::bind(&DStarLiteNode::publish_offboard_mode, this));
            waypoint_timer_ = create_wall_timer(100ms, std::bind(&DStarLiteNode::publish_next_waypoint, this));
        }
    
    private:
        std::shared_ptr<DStarLite3D> planner_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr waypoint_pub_;
        rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
        rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_pub_;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
        rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr setpoint_sub_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
        rclcpp::TimerBase::SharedPtr offboard_timer_;
        rclcpp::TimerBase::SharedPtr waypoint_timer_;
    
        Node3D current_pos_;
        std::vector<Node3D> path_;
        size_t waypoint_index_ = 0;
    
        void odom_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
            current_pos_ = {int(msg->position[0]), int(msg->position[1]), int(msg->position[2])};
        }
    
        void manual_goal_cb(const geometry_msgs::msg::Point::SharedPtr msg) {
            Node3D goal = {int(std::round(msg->x)), int(std::round(msg->y)), int(std::round(msg->z))};
            planner_->set_start(current_pos_);
            planner_->set_goal(goal);
            planner_->compute_shortest_path();
            path_ = planner_->get_path();
            waypoint_index_ = 0;
            publish_path();
            switch_to_offboard_mode();
        }
    
        void publish_next_waypoint() {
            if (waypoint_index_ >= path_.size()) return;
        
            Node3D wp = path_[waypoint_index_];
            px4_msgs::msg::TrajectorySetpoint sp;
            sp.position[0] = std::get<0>(wp);
            sp.position[1] = std::get<1>(wp);
            sp.position[2] = std::get<2>(wp);
            sp.yaw = 0.0;
            sp.timestamp = this->now().nanoseconds() / 1000;
            waypoint_pub_->publish(sp);
        }
        
    
        void publish_path() {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = "map";
            for (const auto& node : path_) {
                geometry_msgs::msg::PoseStamped pose;
                pose.pose.position.x = std::get<0>(node);
                pose.pose.position.y = std::get<1>(node);
                pose.pose.position.z = std::get<2>(node);
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
        }
    
        void publish_offboard_mode() {
            px4_msgs::msg::OffboardControlMode msg;
            msg.position = true;    // or set velocity = true if using velocity-based control
            msg.velocity = false;
            msg.acceleration = false;
            msg.attitude = false;
            msg.body_rate = false;
            msg.timestamp = this->now().nanoseconds() / 1000;
            offboard_pub_->publish(msg);
        }
        
    
        void switch_to_offboard_mode() {
            px4_msgs::msg::VehicleCommand cmd;
            cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
            cmd.param1 = 1.0;   // Custom mode
            cmd.param2 = 6.0;   // PX4 OFFBOARD mode
            cmd.target_system = 1;
            cmd.target_component = 1;
            cmd.source_system = 1;
            cmd.source_component = 1;
            cmd.from_external = true;
            cmd.timestamp = this->now().nanoseconds() / 1000;
            vehicle_cmd_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD MODE command");
        }
        
        void arm_cb(const std_srvs::srv::SetBool::Request::SharedPtr req,
            const std_srvs::srv::SetBool::Response::SharedPtr res) {
            for (int i = 0; i < 10; ++i) {
                publish_offboard_mode();
                publish_next_waypoint();
                rclcpp::sleep_for(50ms);
            }

            px4_msgs::msg::VehicleCommand cmd;
            cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
            cmd.param1 = req->data ? 1.0 : 0.0;
            cmd.param2 = 21196.0;  // Force arm/disarm
            cmd.target_system = 1;
            cmd.target_component = 1;
            cmd.source_system = 1;
            cmd.source_component = 1;
            cmd.from_external = true;
            cmd.timestamp = this->now().nanoseconds() / 1000;
            vehicle_cmd_pub_->publish(cmd);

            res->success = true;
            res->message = req->data ? "Drone armed" : "Drone disarmed";
        }

        void land_cb(const std_srvs::srv::Trigger::Request::SharedPtr req,
                    const std_srvs::srv::Trigger::Response::SharedPtr res) {
        (void)req;
        px4_msgs::msg::VehicleCommand cmd;
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        cmd.param1 = 0.0;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->now().nanoseconds() / 1000;
        vehicle_cmd_pub_->publish(cmd);
        res->success = true;
        res->message = "Landing command sent.";
        }

    };
    
    int main(int argc, char** argv) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<DStarLiteNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }