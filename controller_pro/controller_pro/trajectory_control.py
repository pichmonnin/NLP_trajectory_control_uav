import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from std_srvs.srv import Trigger, SetBool
from geometry_msgs.msg import Point
from gekko import GEKKO

class OffboardControl(Node):
    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        
        # Create services
        self.arm_service = self.create_service(SetBool, "arm_drone", self.arm_callback)
        self.land_service = self.create_service(Trigger, "land_drone", self.land_callback)

        # Create subscribers
        self.goal_sub = self.create_subscription(Point, "/manual_setpoint", self.manual_goal_callback, qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.dt = 0.05
        self.wait_time = int(5 / self.dt)  # Convert to integer for counter comparison
        self.goal_points = [(5.0, 2.0, -3.0), (10.0, 5.0, -5.0) , (2.0 , 3.0 ,-6.0)]  # Ensure all numbers are floats
        self.goal_counter = 0
        self.goal_wait_counter = 0  # Initialize to 0
        self.goal_reached = False
        self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
        self.navigation_timeout = 0 
        self.max_navigation_time = int(20 / self.dt)  # Convert to integer
        
        # State machine variables
        self.state = "IDLE"  # IDLE -> TAKEOFF -> ORIENT -> READY -> WAYPOINT_NAV -> GOAL_REACHED
        self.takeoff_height = -2.0  # Negative because NED frame (2 meters above ground)
        
        # Waypoint navigation variables
        self.waypoints = None  # Initialize as None instead of False for clarity
        self.waypoint_counter = 0  # Start from 0 for list indexing
        self.waypoint_velocities = []
        self.waypoint_accelerations = []
        
        # Current setpoints
        self.waypoint = [0.0, 0.0, 0.0]
        self.waypoint_velocity = [0.0, 0.0, 0.0]
        self.waypoint_acceleration = [0.0, 0.0, 0.0]
        self.yaw_desired = 0.0

        # Create timers
        self.controller_timer = self.create_timer(self.dt, self.controller)
        self.trajectory_planner_timer = self.create_timer(self.dt, self.trajectory_generator)

    def manual_goal_callback(self, msg):
        """Callback for receiving manual goal positions."""
        self.goal_points.append((float(msg.x), float(msg.y), float(msg.z)))  # Ensure float conversion
        if self.state == "IDLE" and not self.goal_reached:
            self.goal_counter = len(self.goal_points) - 1 
            self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
            self.get_logger().info(f"Manual goal received and set as current target: {msg.x}, {msg.y}, {msg.z}")
        else:
            self.get_logger().info(f"Manual goal added to queue: {msg.x}, {msg.y}, {msg.z}")

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        self.vehicle_status = vehicle_status

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')
        self.state = "TAKEOFF"
        self.get_logger().info("Starting takeoff sequence")

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')
        self.state = "IDLE"

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")
        self.state = "IDLE"
        
    def arm_callback(self, request, response):
        """Service callback for arming/disarming the drone."""
        if request.data:
            self.arm()
            response.success = True
            response.message = "Drone Armed"
        else:
            self.disarm()
            response.success = True
            response.message = "Disarm Drone"
        return response
        
    def land_callback(self, request, response):
        """Service callback for landing the drone."""
        self.land()
        response.success = True
        response.message = "Landing triggered"
        return response

    def trajectory_generator(self):        
        if self.state == "READY" and self.waypoints is None:
            # Define the initial conditions
            X0 = float(self.vehicle_local_position.x)
            Y0 = float(self.vehicle_local_position.y)
            Z0 = float(self.vehicle_local_position.z)
            vx0, vy0, vz0 = 0.0, 0.0, 0.0

            self.get_logger().info(f"Optimizer starting position: {X0}, {Y0}, {Z0}")

            # Define the final conditions
            Xf = float(self.x_goal)
            Yf = float(self.y_goal)
            Zf = float(self.z_goal)

            # Set the limits
            v_max, v_min = 3.0, -3.0
            a_max, a_min = 1.0, -1.0

            m = GEKKO(remote=False)

            # Define the time points
            steps = min(int(((X0-Xf)**2 + (Y0-Yf)**2 + (Z0-Zf)**2)**0.5) + 5, 10)
            T = 5.0
            m.time = np.linspace(0, T, steps)

            # Define the state variables
            X = m.Var(value=X0)
            Y = m.Var(value=Y0)
            Z = m.Var(value=Z0)
            vx = m.Var(value=vx0, lb=v_min, ub=v_max)
            vy = m.Var(value=vy0, lb=v_min, ub=v_max)
            vz = m.Var(value=vz0, lb=v_min, ub=v_max)

            # Define the control variables
            ax = m.MV(value=0.0, lb=a_min, ub=a_max)
            ay = m.MV(value=0.0, lb=a_min, ub=a_max)
            az = m.MV(value=0.0, lb=a_min, ub=a_max)
            ax.STATUS, ay.STATUS, az.STATUS = 1, 1, 1            

            tf = m.FV(value=1.0, lb=0.1, ub=T)
            tf.STATUS = 1

            # Define the differential equations
            m.Equation(X.dt() == vx*tf)
            m.Equation(Y.dt() == vy*tf)
            m.Equation(Z.dt() == vz*tf)

            m.Equation(vx.dt() == ax*tf)
            m.Equation(vy.dt() == ay*tf)
            m.Equation(vz.dt() == az*tf)

            # Define the final conditions
            m.fix(X, pos=len(m.time)-1, val=Xf)
            m.fix(Y, pos=len(m.time)-1, val=Yf)
            m.fix(Z, pos=len(m.time)-1, val=Zf)
            m.fix(vx, pos=len(m.time)-1, val=0.0)
            m.fix(vy, pos=len(m.time)-1, val=0.0)
            m.fix(vz, pos=len(m.time)-1, val=0.0)

            # Calculate the distance between the initial and final points
            distance = np.sqrt((Xf-X0)**2 + (Yf-Y0)**2 + (Zf-Z0)**2)
            velocity = 0.5  # Set the speed to reach the final point
            tf_scheduled = distance / velocity  # Scheduled time to reach final conditions

            A = 1.0  # Weight for prioritizing faster trajectories
            mayers_term = A*(tf-tf_scheduled)**2
            cost = mayers_term

            # Define the objective
            m.Obj(cost)

            # Solve the optimization problem
            m.options.IMODE = 6
            m.options.SOLVER = 1
            m.solve(disp=False)

            self.get_logger().info(f'Minimum Cost: {m.options.OBJFCNVAL}')
            self.get_logger().info(f'Final Time: {tf.value[0]}')

            # Generate waypoints from the solution
            self.waypoints = [(X[0], Y[0], Z[0])]
            steps = min(int(distance) + 2, 10)

            for i in range(len(X)-1):
                x = np.linspace(X[i], X[i+1], steps)[1:]
                y = np.linspace(Y[i], Y[i+1], steps)[1:]
                z = np.linspace(Z[i], Z[i+1], steps)[1:]

                if (x[0], y[0], z[0]) != self.waypoints[-1]:
                    self.waypoints += [(x[j], y[j], z[j]) for j in range(len(x))]
            
            self.get_logger().info(f"Number of Waypoints: {len(self.waypoints)}")
            
            # Generate velocity setpoints
            dt = tf_scheduled / len(self.waypoints)
            self.waypoint_velocities = []
            for i in range(len(self.waypoints)-1):
                x = (self.waypoints[i+1][0] - self.waypoints[i][0]) / dt
                y = (self.waypoints[i+1][1] - self.waypoints[i][1]) / dt
                z = (self.waypoints[i+1][2] - self.waypoints[i][2]) / dt
                self.waypoint_velocities.append((x, y, z))
            
            # Pad the last velocity with zeros
            while len(self.waypoint_velocities) < len(self.waypoints):
                self.waypoint_velocities.append((0.0, 0.0, 0.0))
            
            self.get_logger().info(f"Number of Velocities: {len(self.waypoint_velocities)}")
                
            # Generate acceleration setpoints
            self.waypoint_accelerations = []
            for i in range(len(self.waypoint_velocities)-1):
                x = (self.waypoint_velocities[i+1][0] - self.waypoint_velocities[i][0]) / dt
                y = (self.waypoint_velocities[i+1][1] - self.waypoint_velocities[i][1]) / dt
                z = (self.waypoint_velocities[i+1][2] - self.waypoint_velocities[i][2]) / dt
                self.waypoint_accelerations.append((x, y, z))
            
            # Pad the last acceleration with zeros
            while len(self.waypoint_accelerations) < len(self.waypoints):
                self.waypoint_accelerations.append((0.0, 0.0, 0.0))
            
            self.get_logger().info(f"Number of Accelerations: {len(self.waypoint_accelerations)}")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, position: list, velocity: list, acceleration: list, yaw: float):
        """Publish the trajectory setpoint."""
        current_pos = (self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z)
        self.get_logger().info(f"Current Position: {current_pos}")
        self.get_logger().info(f"Publishing velocity: {np.round(velocity, 2)}")
        self.get_logger().info(f"Publishing acceleration: {np.round(acceleration, 2)}")
        self.get_logger().info(f"Publishing yaw: {np.round(yaw, 2)}")
        
        goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
        self.get_logger().info(f'Goal Distance: {goal_distance}')

        msg = TrajectorySetpoint()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        msg.yaw = yaw 
        msg.yawspeed = 0.1
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def dist2wp(self, wp_x, wp_y, wp_z) -> float:
        """Calculate Euclidean distance to the waypoint."""
        return float((wp_x - self.vehicle_local_position.x)**2 +
                    (wp_y - self.vehicle_local_position.y)**2 + 
                    (wp_z - self.vehicle_local_position.z)**2)**0.5

    def controller(self) -> None:
        """Main controller callback function."""
        self.publish_offboard_control_heartbeat_signal()
        
        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
        
        self.offboard_setpoint_counter += 1
        
        # Only proceed if in offboard mode
        if self.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            return

        # State machine logic
        if self.state == "IDLE":
            if self.goal_counter < len(self.goal_points) and self.waypoints is None and not self.goal_reached:
                if self.goal_counter == 0:
                    self.get_logger().info("Mission start - taking off")
                    self.state = "TAKEOFF"
                else:
                    self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
                    self.state = "ORIENT"
            elif self.goal_counter >= len(self.goal_points):
                self.get_logger().info("All goals completed. Mission ending")
                self.land()
                self.disarm()

        elif self.state == "TAKEOFF":
            takeoff_height_reached = abs(self.vehicle_local_position.z - self.takeoff_height) < 0.2
            
            if takeoff_height_reached:
                self.get_logger().info("Takeoff complete, switching to ORIENT state")
                self.state = "ORIENT"
            else:
                self.waypoint = [self.vehicle_local_position.x, 
                                self.vehicle_local_position.y, 
                                self.takeoff_height]
                self.waypoint_velocity = [0.0, 0.0, -0.5]
                self.waypoint_acceleration = [0.0, 0.0, 0.0]
                self.yaw_desired = self.vehicle_local_position.heading

        elif self.state == "ORIENT":
            self.yaw_desired = np.arctan2(self.y_goal - self.vehicle_local_position.y, 
                                         self.x_goal - self.vehicle_local_position.x)
            
            self.waypoint = [self.vehicle_local_position.x, 
                            self.vehicle_local_position.y, 
                            self.vehicle_local_position.z]
            self.waypoint_velocity = [0.0, 0.0, 0.0]
            self.waypoint_acceleration = [0.0, 0.0, 0.0]
            
            yaw_error = abs(self.yaw_desired - self.vehicle_local_position.heading)
            yaw_error = min(yaw_error, 2*np.pi - yaw_error)
            
            if yaw_error < 0.1:
                self.get_logger().info("Orientation complete, switching to READY state")
                self.state = "READY"

        elif self.state == "READY":
            if self.waypoints is None:
                self.waypoint = [float(self.vehicle_local_position.x), 
                                 float(self.vehicle_local_position.y), 
                                 float(self.vehicle_local_position.z)]
                self.waypoint_velocity = [0.0, 0.0, 0.0]
                self.waypoint_acceleration = [0.0, 0.0, 0.0]
            else:
                self.get_logger().info("Trajectory generated, switching to WAYPOINT_NAV state")
                self.state = "WAYPOINT_NAV"

        elif self.state == "WAYPOINT_NAV":
            self.navigation_timeout += 1 
            
            if self.navigation_timeout > self.max_navigation_time:
                goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
                if goal_distance < 2.0:
                    self.waypoint = [float(self.vehicle_local_position.x),
                                    float(self.vehicle_local_position.y),
                                    float(self.vehicle_local_position.z)]
                    self.waypoint_counter = 0
                    self.state = "GOAL_REACHED"
                    self.goal_reached = True
                else:
                    self.waypoint = [float(self.vehicle_local_position.x),
                                    float(self.vehicle_local_position.y),
                                    float(self.vehicle_local_position.z)]
                    self.state = "READY"
                self.navigation_timeout = 0
                return
            
            if self.waypoints:
                distance_to_waypoint = self.dist2wp(float(self.waypoints[self.waypoint_counter][0]), 
                                                  float(self.waypoints[self.waypoint_counter][1]), 
                                                  float(self.waypoints[self.waypoint_counter][2]))

                if distance_to_waypoint < 0.5 and self.waypoint_counter < len(self.waypoints)-1:
                    self.waypoint_counter += 1
                
                self.yaw_desired = np.arctan2(self.waypoints[self.waypoint_counter][1] - self.waypoints[max(0, self.waypoint_counter-1)][1], 
                                             self.waypoints[self.waypoint_counter][0] - self.waypoints[max(0, self.waypoint_counter-1)][0])
                
                self.waypoint = [float(self.waypoints[self.waypoint_counter][0]),
                                float(self.waypoints[self.waypoint_counter][1]),
                                float(self.waypoints[self.waypoint_counter][2])]
                
                self.waypoint_velocity = [float(self.waypoint_velocities[self.waypoint_counter][0]),
                                         float(self.waypoint_velocities[self.waypoint_counter][1]), 
                                         float(self.waypoint_velocities[self.waypoint_counter][2])]
                
                self.waypoint_acceleration = [float(self.waypoint_accelerations[self.waypoint_counter][0]),
                                             float(self.waypoint_accelerations[self.waypoint_counter][1]), 
                                             float(self.waypoint_accelerations[self.waypoint_counter][2])]

                goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
                if goal_distance < 1.0 or self.waypoint_counter == len(self.waypoints)-1:
                    self.waypoints = None
                    self.waypoint_counter = 0
                    self.state = "GOAL_REACHED"
                    self.goal_reached = True

        elif self.state == "GOAL_REACHED":
            self.goal_wait_counter += 1

            self.waypoint = [float(self.vehicle_local_position.x), 
                            float(self.vehicle_local_position.y), 
                            float(self.vehicle_local_position.z)]
            self.waypoint_velocity = [0.0, 0.0, 0.0]
            self.waypoint_acceleration = [0.0, 0.0, 0.0]

            if self.goal_wait_counter > self.wait_time:
                self.goal_wait_counter = 0
                self.goal_reached = False
                self.goal_counter += 1
                if self.goal_counter < len(self.goal_points):
                    self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
                    self.state = "ORIENT"
                else:
                    self.state = "IDLE"

        # Publish the position setpoint
        self.publish_position_setpoint(self.waypoint, self.waypoint_velocity,
                                     self.waypoint_acceleration, self.yaw_desired)

def main(args=None) -> None:
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()