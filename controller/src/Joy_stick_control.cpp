#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <chrono>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

class JoystickMotorControl : public rclcpp::Node {
public:
    JoystickMotorControl() : Node("joystick_motor_control") {
        // Initialize parameters that can be set from launch file
        this->declare_parameter("max_velocity_xy", 5.0);  // m/s
        this->declare_parameter("max_velocity_z", 3.0);   // m/s
        this->declare_parameter("max_yaw_rate", 1.0);     // rad/s
        
        max_velocity_xy_ = this->get_parameter("max_velocity_xy").as_double();
        max_velocity_z_ = this->get_parameter("max_velocity_z").as_double();
        max_yaw_rate_ = this->get_parameter("max_yaw_rate").as_double();
        
        // QoS profile for PX4 communication
        rclcpp::QoS qos_profile(rclcpp::QoS(10).best_effort().transient_local());
        
        // Publishers
        pos_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", qos_profile);
        cmd_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", qos_profile);
        offboard_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", qos_profile);

        // Subscribers
        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", qos_profile, std::bind(&JoystickMotorControl::vehicle_status_callback, this, _1));

        // QoS for joystick input
        rclcpp::QoS joy_qos(10);
        joy_qos.best_effort();
        joy_qos.keep_last(10);
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", joy_qos, std::bind(&JoystickMotorControl::joy_callback, this, _1));

        // Initialize state variables
        armed_ = false;
        offboard_mode_ = false;
        velocity_ = {0.0, 0.0, 0.0};  // X, Y, Z velocity
        yaw_rate_ = 0.0;
        last_command_time_ = this->now();

        // Timers
        timer_ = this->create_wall_timer(20ms, std::bind(&JoystickMotorControl::send_velocity_command, this));
        pre_arm_timer_ = this->create_wall_timer(500ms, std::bind(&JoystickMotorControl::pre_arm_callback, this));
        safety_timer_ = this->create_wall_timer(1000ms, std::bind(&JoystickMotorControl::safety_check, this));

        RCLCPP_INFO(this->get_logger(), "Joystick Motor Control Ready!");
        RCLCPP_INFO(this->get_logger(), "Joystick Controls:");
        RCLCPP_INFO(this->get_logger(), "  Left stick: X/Y translation");
        RCLCPP_INFO(this->get_logger(), "  Right stick (vertical): Z velocity (up/down)");
        RCLCPP_INFO(this->get_logger(), "  Right stick (horizontal): Yaw control");
        RCLCPP_INFO(this->get_logger(), "  Button 6 (LB): Arm drone");
        RCLCPP_INFO(this->get_logger(), "  Button 1 (B): Disarm drone");
    }

private:
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr pos_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr pre_arm_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;

    std::vector<float> velocity_;
    float yaw_rate_;
    bool armed_, offboard_mode_;
    double max_velocity_xy_, max_velocity_z_, max_yaw_rate_;
    rclcpp::Time last_command_time_;

    void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        armed_ = (msg->arming_state == 2);
        offboard_mode_ = (msg->nav_state == 14);
        
        // Log state changes
        static bool last_armed_state = false;
        static bool last_offboard_state = false;
        
        if (armed_ != last_armed_state) {
            RCLCPP_INFO(this->get_logger(), "Drone armed state: %s", armed_ ? "ARMED" : "DISARMED");
            last_armed_state = armed_;
        }
        
        if (offboard_mode_ != last_offboard_state) {
            RCLCPP_INFO(this->get_logger(), "Offboard mode: %s", offboard_mode_ ? "ACTIVE" : "INACTIVE");
            last_offboard_state = offboard_mode_;
        }
    }

    void pre_arm_callback() {
        publish_offboard_mode();
    }
    
    void safety_check() {
        // Check for joystick timeouts and safety measures
        auto elapsed = (this->now() - last_command_time_).seconds();
        if (elapsed > 1.0 && armed_) {
            // If no joystick update for more than 1 second and drone is armed,
            // set velocities to 0 to hover in place (failsafe)
            velocity_ = {0.0, 0.0, 0.0};
            yaw_rate_ = 0.0;
            RCLCPP_WARN(this->get_logger(), "Joystick timeout detected! Hovering in place.");
        }
    }

    void publish_offboard_mode() {
        auto msg = px4_msgs::msg::OffboardControlMode();
        msg.position = false;
        msg.velocity = true;  // Use velocity-based control
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = true;  // Enable yaw rate control
        msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        offboard_pub_->publish(msg);
    }

    void send_velocity_command() {
        // Publish offboard control mode before every command for reliability
        publish_offboard_mode();
        
        // Send trajectory setpoint with velocity commands
        auto msg = px4_msgs::msg::TrajectorySetpoint();
        msg.velocity[0] = velocity_[0];  // X velocity
        msg.velocity[1] = velocity_[1];  // Y velocity
        msg.velocity[2] = velocity_[2];  // Z velocity
        msg.yawspeed = yaw_rate_;       // Yaw rate control
        msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        pos_pub_->publish(msg);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Update timestamp to track joystick activity
        last_command_time_ = this->now();
        
        // Map joystick axes to velocity commands
        // Left stick: X/Y translation
        float x_velocity = msg->axes[0] * max_velocity_xy_;  // Left/right
        float y_velocity = msg->axes[1] * max_velocity_xy_;  // Forward/backward
        
        // Right stick: Z velocity (up/down) and yaw control
        float z_velocity = -msg->axes[3] * max_velocity_z_;  // Up/down (negative because Z is down in NED)
        float yaw = msg->axes[2] * max_yaw_rate_;           // Yaw rotation
        
        // Update velocity vector and yaw rate
        velocity_ = {
            y_velocity,  // X in body frame (forward/backward)
            x_velocity,  // Y in body frame (left/right)
            z_velocity   // Z (altitude change)
        };
        yaw_rate_ = yaw;
        
        // Process button presses with debouncing
        static bool arm_pressed = false;
        static bool disarm_pressed = false;
        
        if (msg->buttons[6] && !arm_pressed && !armed_) {
            arm_drone();
            arm_pressed = true;
        } else if (!msg->buttons[6]) {
            arm_pressed = false;
        }
        
        if (msg->buttons[1]) {
            disarm_drone();
        } 
    }

    void arm_drone() {
        RCLCPP_INFO(this->get_logger(), "Arming drone...");
        
        // Send multiple offboard control mode messages to ensure mode switch
        for (int i = 0; i < 10; i++) {
            publish_offboard_mode();
            rclcpp::sleep_for(50ms);
        }

        // Send arm command
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0;  // 1 to arm
        msg.param2 = 21196.0;  // Force arm/disarm (magic number from PX4)
        msg.target_system = 1;  // System ID (fixed for this example)
        msg.target_component = 1;  // Component ID (fixed for this example)
        msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        cmd_pub_->publish(msg);
        
        // Also send the command to engage offboard mode
        auto offboard_msg = px4_msgs::msg::VehicleCommand();
        offboard_msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        offboard_msg.param1 = 1.0;  // Custom mode
        offboard_msg.param2 = 6.0;  // Custom submode - offboard
        offboard_msg.target_system = 1;
        offboard_msg.target_component = 1;
        offboard_msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        cmd_pub_->publish(offboard_msg);
    }

    void disarm_drone() {
        RCLCPP_INFO(this->get_logger(), "Disarming drone...");
        auto msg = px4_msgs::msg::VehicleCommand();
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 0.0;  // 0 to disarm
        msg.param2 = 21196.0;  // Force arm/disarm
        msg.target_system = 1;
        msg.target_component = 1;
        msg.timestamp = static_cast<uint64_t>(this->now().nanoseconds() / 1000);
        cmd_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoystickMotorControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}