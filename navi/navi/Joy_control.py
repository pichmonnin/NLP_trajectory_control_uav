import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy, Imu
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32 
import numpy as np
from px4_msgs.msg import TrajectorySetpoint, VehicleCommand, OffboardControlMode, VehicleStatus # type: ignore
import time

class JoystickDroneControl(Node):
    def __init__(self):
        super().__init__('joystick_drone_control')
        
        # Create PX4-compatible QoS profile
        # PX4 uses best effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers for PX4 control (using PX4 QoS)
        self.pos_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.offboard_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        
        # Subscriber for vehicle status (using PX4 QoS)
        self.status_sub = self.create_subscription(
            VehicleStatus, 
            '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 
            qos_profile
        )  
        self.ekf_z_sub = self.create_subscription(
            Pose,
            '/ekf_pose',
            self.kf_pose_callback,
            qos_profile
        )
        
        # Subscribe to joystick inputs (standard QoS is fine for joystick)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Drone State
        self.estimated_x, self.estimated_y, self.estimated_z = 0.0, 0.0, 0.0
        self.estimated_roll, self.estimated_pitch, self.estimated_yaw = 0.0, 0.0, 0.0
        self.target_x, self.target_y, self.target_z = 0.0, 0.0, 0.0
        self.target_yaw = 0.0
        self.position_initialized = False
        self.yaw_initialized = False
        self.step = 0.1  # Smaller step size for finer control
        self.armed = False
        self.offboard_mode = False
        self.is_ready = False
        
        # Keep track of last joystick command time to prevent multiple button presses
        self.last_button_time = {}
        self.last_status_log_time = time.time()
        
        # Position limits for safety
        self.max_xy = 10.0  # Maximum 10m in any horizontal direction
        self.min_z = -5.0   # Maximum 5m altitude
        self.max_z = 0.0    # Don't go underground
        
        # Create a timer for publishing control mode at 50Hz (PX4 requires high rate)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        # Create initial setpoint to establish control before arming
        self.initial_setpoint_sent = False
        self.pre_arm_timer = self.create_timer(1.0, self.pre_arm_callback)
        
        self.get_logger().info("Joystick Drone Control Ready! Use your joystick to fly.")
        
    def kf_pose_callback(self, msg):
        """Process pose data from EKF"""
        self.estimated_x = msg.position.x
        self.estimated_y = msg.position.y
        self.estimated_z = msg.position.z 
        
        # Initialize target positions with current position (only once)
        if not self.position_initialized:
            self.target_x = self.estimated_x
            self.target_y = self.estimated_y
            self.target_z = self.estimated_z
            self.position_initialized = True
            self.get_logger().info(f"Initial position set to X: {self.target_x:.2f}, Y: {self.target_y:.2f}, Z: {self.target_z:.2f}")
        
        roll, pitch, yaw = self.quaternion_to_euler(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.estimated_roll = roll
        self.estimated_pitch = pitch 
        self.estimated_yaw = yaw
        
        if not self.yaw_initialized:
            self.target_yaw = self.estimated_yaw
            self.yaw_initialized = True
            self.get_logger().info(f"Initial yaw set to: {self.target_yaw:.2f}")

        self.get_logger().debug(f"KF Pose -> X: {self.estimated_x:.2f}, Y: {self.estimated_y:.2f}, Z: {self.estimated_z:.2f}, Yaw: {self.estimated_yaw:.2f}")

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(t3, t4)
        return roll, pitch, yaw
        
    def pre_arm_callback(self):
        """Send initial setpoints before arming to satisfy PX4 requirements"""
        self.publish_offboard_mode()
        self.send_setpoint()
        self.initial_setpoint_sent = True
        self.get_logger().info("Sending initial setpoints...")
        
        # After 5 seconds, cancel this timer
        if self.initial_setpoint_sent:
            self.pre_arm_timer.cancel()
            self.is_ready = True
            self.get_logger().info("Ready to arm! Press X button to arm the drone.")
    
    def vehicle_status_callback(self, msg):
        """Monitor vehicle status for arm state and flight mode"""
        # Log the message to confirm we're receiving it
        self.get_logger().debug(f"Vehicle Status - arming_state: {msg.arming_state}, nav_state: {msg.nav_state}")
        
        old_armed = self.armed if hasattr(self, 'armed') else False
        old_offboard = self.offboard_mode if hasattr(self, 'offboard_mode') else False
        
        self.armed = msg.arming_state == 2  # 2 is armed
        
        # Check if in offboard mode (mode 14 in PX4 nav_state)
        nav_state = msg.nav_state
        self.offboard_mode = (nav_state == 14)  # 14 is offboard mode
        
        # Log state changes
        if old_armed != self.armed:
            self.get_logger().info(f"Arm state changed: {self.armed}")
            
        if old_offboard != self.offboard_mode:
            self.get_logger().info(f"Offboard mode changed: {self.offboard_mode}")
    
    def timer_callback(self):
        """Publish offboard control mode and setpoints at regular intervals"""
        # Always publish offboard mode to make PX4 happy
        self.publish_offboard_mode()
        
        # Always send setpoints for debugging
        self.send_setpoint()
        
        # Log status periodically
        if time.time() - self.last_status_log_time > 5.0:  # Log every 5 seconds
            self.get_logger().info(f"Status - Armed: {self.armed}, Offboard: {self.offboard_mode}, "
                                  f"Target position: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}")
            self.last_status_log_time = time.time()
    
    def publish_offboard_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        # Use microseconds timestamp as expected by PX4
        msg.timestamp = int(time.time() * 1000000)
        self.offboard_pub.publish(msg)
    
    def arm_drone(self):
        """Arm the drone"""
        if not self.is_ready:
            self.get_logger().warn("Not ready to arm yet. Waiting for initial setpoints...")
            return
            
        self.get_logger().info("Arming drone...")
        
        # Send multiple setpoints before arming (PX4 needs these)
        for i in range(10):
            self.publish_offboard_mode()
            self.send_setpoint()
            time.sleep(0.05)
            
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0  # Arm
        msg.param2 = 21196.0    # Force Ararming
        msg.target_system = 1
        msg.target_component = 1
        msg.timestamp = int(time.time() * 1000000)
        self.cmd_pub.publish(msg)
        self.get_logger().info("Arm command sent!")
        time.sleep(1.0)
        self.enter_offboard_mode()
    
    def enter_offboard_mode(self):
        """Switch to offboard control mode"""
        self.get_logger().info("Entering offboard mode...")
        
        # Send multiple setpoints before switching mode
        for i in range(10):
            self.publish_offboard_mode()
            self.send_setpoint()
            time.sleep(0.05)
            
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0  # Custom mode
        msg.param2 = 6.0  # Offboard mode
        msg.target_system = 1
        msg.target_component = 1
        msg.timestamp = int(time.time() * 1000000)
        self.cmd_pub.publish(msg)
        self.get_logger().info("Offboard command sent!")
    
    def disarm_drone(self):
        """Disarm the drone"""
        self.get_logger().info("Disarming drone...")
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # Disarm
        msg.target_system = 1
        msg.target_component = 1
        msg.timestamp = int(time.time() * 1000000)
        self.cmd_pub.publish(msg)
    
    def takeoff(self, altitude=-2.0):
        """Command the drone to takeoff to specified altitude"""
        # Force arming if needed
        if not self.armed:
            self.get_logger().info("Attempting to arm before takeoff...")
            self.arm_drone()
            time.sleep(1.0)  # Give more time for arm command to process
            
        # Force offboard mode if needed
        if not self.offboard_mode and self.armed:
            self.get_logger().info("Attempting to enter offboard mode before takeoff...")
            self.enter_offboard_mode()
            time.sleep(1.0)  # Give more time for mode change to process
            
        self.get_logger().info(f"Taking off to {-altitude} meters...")
        # Set the takeoff altitude (using current X and Y position)
        self.target_z = altitude
        # Make sure we're not moving horizontally during takeoff
        self.target_x = self.estimated_x
        self.target_y = self.estimated_y
        self.send_setpoint()
    
    def land(self):
        """Command the drone to land"""
        if not self.offboard_mode:
            self.get_logger().warn("Drone is not in Offboard Mode! Attempting to switch")
            self.enter_offboard_mode()
            time.sleep(0.5)
        
        self.get_logger().info("Landing...")
        
        # First command a controlled descent to zero
        self.target_z = 0.0  # Set target Z to ground level
        self.send_setpoint()
        
        # Then send the land command
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        msg.target_system = 1
        msg.target_component = 1
        msg.timestamp = int(time.time() * 1000000)
        self.cmd_pub.publish(msg)
        
        if abs(self.estimated_z) < 0.05:
            self.get_logger().info("Drone is near ground level. Sending disarm command.")
            self.disarm_drone()
    
    def emergency_stop(self):
        """Emergency stop - disarm immediately"""
        self.get_logger().error("EMERGENCY STOP ACTIVATED")
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 0.0  # Disarm
        msg.param2 = 21196.0  # Force disarm (magic number from PX4)
        msg.target_system = 1
        msg.target_component = 1
        msg.timestamp = int(time.time() * 1000000)
        self.cmd_pub.publish(msg)
    
    def button_debounce(self, button_idx):
        """Prevent multiple actions from a single button press"""
        current_time = time.time()
        if button_idx in self.last_button_time:
            if current_time - self.last_button_time[button_idx] < 0.5:  # 500ms debounce time
                return False
        
        self.last_button_time[button_idx] = current_time
        return True
    
    def send_setpoint(self):
        """Publish position setpoint to PX4"""
        msg = TrajectorySetpoint()
        msg.position = [self.target_x, self.target_y, self.target_z]
        msg.yaw = self.target_yaw
        msg.velocity = [0.0, 0.0, 0.0]  
        msg.timestamp = int(time.time() * 1000000)
        self.pos_pub.publish(msg)
        
        self.get_logger().debug(f"Setpoint: X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}, Yaw={self.target_yaw:.2f}")
    
    def joy_callback(self, msg: Joy):
        """Process joystick input"""
        # Log joystick input for debugging
        self.get_logger().debug(f"Joy input: axes={msg.axes}, buttons={msg.buttons}")
        
        # Process movement commands
        # Axes Mapping (for Xbox Controller)
        y_cmd = msg.axes[1] * self.step  # Forward/Backward (Left Stick Y)
        x_cmd = msg.axes[0] * self.step  # Left/Right (Left Stick X)
        yaw_cmd = msg.axes[2] * 0.05     # Rotate Left/Right (Right Stick X)
        z_cmd = msg.axes[3] * -self.step # Up/Down (Right Stick Y)
        
        self.target_x = np.clip(self.target_x + x_cmd, -self.max_xy, self.max_xy)
        self.target_y = np.clip(self.target_y + y_cmd, -self.max_xy, self.max_xy)
        self.target_yaw += yaw_cmd
        self.target_z = np.clip(self.target_z + z_cmd, self.min_z, self.max_z)
        
        # Only log when there's actual movement
        if abs(x_cmd) > 0.01 or abs(y_cmd) > 0.01 or abs(z_cmd) > 0.01 or abs(yaw_cmd) > 0.01:
            self.get_logger().info(f"Moving to X={self.target_x:.2f}, Y={self.target_y:.2f}, Z={self.target_z:.2f}, Yaw={self.target_yaw:.2f}")
        
        # Button handling
        if msg.buttons[1] == 1 and self.button_debounce(1):
            self.get_logger().info("B button pressed: Land")
            self.land()
        if msg.buttons[6] == 1 and self.button_debounce(6):
            self.get_logger().info("LB Button Pressed: Drone Arm")
            self.arm_drone()
        if msg.buttons[2] == 1 and self.button_debounce(2):
            self.get_logger().info("Y button pressed: Disarm")
            self.disarm_drone()
        if msg.buttons[7] == 1 and self.button_debounce(7):
            self.get_logger().info("RB button pressed: Emergency Stop")
            self.emergency_stop()

    
    def test_px4_comms(self):
        """Test basic communication with PX4"""
        self.get_logger().info("Testing PX4 communication...")
        
        # Wait a bit for subscriptions to be established
        time.sleep(2.0)
        
        # Send a few setpoints
        for i in range(10):
            self.publish_offboard_mode()
            self.send_setpoint()
            self.get_logger().info(f"Sent setpoint {i+1}/10")
            time.sleep(0.1)
        
        # Try to arm (but don't take off)
        self.arm_drone()
        time.sleep(2.0)
        
        # Check status
        self.get_logger().info(f"Arm status: {self.armed}, Offboard status: {self.offboard_mode}")
        
        # Disarm
        self.disarm_drone()
        time.sleep(1.0)
        
        self.get_logger().info("PX4 communication test complete")

def main():
    rclpy.init()
    node = JoystickDroneControl()
    
    # Uncomment to run communications test at startup
    # node.test_px4_comms()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()