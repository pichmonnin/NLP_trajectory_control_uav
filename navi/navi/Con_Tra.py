import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus # type: ignore
from std_srvs.srv import Trigger
from navi_interfaces.srv import SetGoal # type: ignore
from example_interfaces.srv import SetBool

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode with multiple services."""

    def __init__(self) -> None:
        super().__init__('offboard_control_with_services')

        # Configure QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Services
        self.create_service(Trigger, 'start_flight', self.start_flight_callback)
        self.create_service(Trigger, 'land_flight', self.land_flight_callback)
        self.create_service(Trigger, 'arm_drone', self.arm_drone_callback)
        self.create_service(Trigger, 'disarm_drone', self.disarm_drone_callback)
        self.create_service(Trigger, 'pause_flight', self.pause_flight_callback)
        self.create_service(Trigger, 'resume_flight', self.resume_flight_callback)
        self.create_service(Trigger, 'rtl_flight', self.rtl_flight_callback)
        self.create_service(SetGoal, 'set_goal', self.set_goal_callback)

        # Flags and Goal
        self.flight_active = False
        self.flight_paused = False
        self.goal_set = False
        self.goal_position = [0.0, 0.0, -3.0]

        # Timer for control loop
        self.controller_timer = self.create_timer(0.05, self.controller)

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    # Service Callbacks
    def start_flight_callback(self, request, response):
        self.flight_active = True
        response.success = True
        response.message = "Flight activated."
        self.get_logger().info('Flight has been activated via service.')
        return response

    def land_flight_callback(self, request, response):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing command sent.')
        response.success = True
        response.message = "Landing initiated."
        return response

    def arm_drone_callback(self, request, response):
        self.arm()
        response.success = True
        response.message = "Drone armed."
        return response

    def disarm_drone_callback(self, request, response):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent.')
        response.success = True
        response.message = "Drone disarmed."
        return response

    def pause_flight_callback(self, request, response):
        self.flight_paused = True
        self.get_logger().info('Flight paused.')
        response.success = True
        response.message = "Flight paused."
        return response

    def resume_flight_callback(self, request, response):
        if self.flight_active:
            self.flight_paused = False
            self.get_logger().info('Flight resumed.')
            response.success = True
            response.message = "Flight resumed."
        else:
            response.success = False
            response.message = "Flight not active. Cannot resume."
        return response

    def rtl_flight_callback(self, request, response):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.get_logger().info('Return to Launch command sent.')
        response.success = True
        response.message = "Returning to launch position."
        return response

    def set_goal_callback(self, request, response):
        self.goal_position = [request.x, request.y, request.z]
        self.goal_set = True
        self.get_logger().info(f"New goal set to: {self.goal_position}")
        response.success = True
        response.message = "Goal position updated"
        return response

    # Command Publishers
    def publish_offboard_control_heartbeat_signal(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, x, y, z, yaw):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Drone armed.')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard mode engaged.')

    # Controller
    def controller(self):
        if self.flight_active and not self.flight_paused:
            self.publish_offboard_control_heartbeat_signal()
            self.engage_offboard_mode()
            self.arm()

            # Fly to the set goal if available
            if self.goal_set:
                self.publish_position_setpoint(*self.goal_position, 0.0)
                self.get_logger().info(f'Flying to set goal: {self.goal_position}')
            else:
                # Default setpoint if no goal is set
                self.publish_position_setpoint(0.0, 0.0, -3.0, 0.0)
                self.get_logger().info('Publishing default takeoff setpoint.')

        elif self.flight_paused:
            self.get_logger().info('Flight is paused.')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
