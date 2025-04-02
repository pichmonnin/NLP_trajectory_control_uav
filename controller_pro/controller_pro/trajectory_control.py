import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus # type: ignore
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
        self.arm_callback = self.create_service(SetBool, "arm_drone", self.arm_callback)
        self.land_callback = self.create_service(Trigger, "land_drone", self.land_callback)

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
        self.wait_time = 5 / self.dt
        self.goal_points = [(5, 2, -3)]  
        self.goal_counter = 0
        self.goal_wait_counter = 1
        self.goal_reached = False
        self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
        self.navigation_timeout = 0 
        self.max_navigation_time = 20 /self.dt
        print('Goal Position: ', self.x_goal, self.y_goal, self.z_goal)

        # Add new state variables for takeoff
        self.state = "IDLE"  # IDLE -> TAKEOFF -> ORIENT -> READY -> WAYPOINT_NAV -> GOAL_REACHED
        self.takeoff_height = -2.0  # Negative because NED frame (2 meters above ground)
        self.takeoff_complete = False
        self.orient_complete = False
        self.waypoints = False
        self.waypoint_counter = 1
        
        # Default waypoint, velocity, and acceleration
        self.waypoint = [0.0, 0.0, 0.0]
        self.waypoint_velocity = [0.0, 0.0, 0.0]
        self.waypoint_acceleration = [0.0, 0.0, 0.0]
        self.yaw_desired = 0.0

        # Create a timer to publish control commands
        self.controller_timer = self.create_timer(self.dt, self.controller)
        self.trajectory_planner_timer = self.create_timer(self.dt, self.trajectory_generator)

    def manual_goal_callback(self, msg):
        self.goal_points = [(msg.x, msg.y, msg.z)]
        self.goal_counter = 0 
        self.goal_reached = False
        self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
        print(f"Manual goal received: {msg.x}, {msg.y}, {msg.z}")

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
        # Start the takeoff sequence upon arming
        self.state = "TAKEOFF"
        print("Starting takeoff sequence")

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
        self.land()
        response.success = True
        response.message = "Landing triggered"
        return response

    def trajectory_generator(self):        
        if self.state =="READY" and not self.waypoints and not self.goal_reached:

            # Define the initial conditions
            X0 = self.vehicle_local_position.x
            Y0 = self.vehicle_local_position.y
            Z0 = self.vehicle_local_position.z
            vx0, vy0, vz0 = 0, 0, 0

            print("Optimizer starting position: ", X0, Y0, Z0)

            # Define the final conditions
            Xf = self.x_goal
            Yf = self.y_goal
            Zf = self.z_goal

            # Set the limits
            v_max, v_min = 3, -3
            a_max, a_min = 1, -1

            m = GEKKO(remote=False)

            # Define the time points
            steps = min(int(((X0-Xf)**2 + (Y0-Yf)**2 + (Z0-Zf)**2)**0.5)+5, 10)
            T = 5
            m.time = np.linspace(0, T, steps)

            # Define the state variables
            X = m.Var(value=X0)
            Y = m.Var(value=Y0)
            Z = m.Var(value=Z0)
            vx = m.Var(value=vx0, lb=v_min, ub=v_max)
            vy = m.Var(value=vy0, lb=v_min, ub=v_max)
            vz = m.Var(value=vz0, lb=v_min, ub=v_max)

            # Define the control variables
            ax = m.MV(value=0, lb=a_min, ub=a_max)
            ay = m.MV(value=0, lb=a_min, ub=a_max)
            az = m.MV(value=0, lb=a_min, ub=a_max)
            ax.STATUS, ay.STATUS, az.STATUS = 1, 1, 1            

            tf = m.FV(value=1, lb=0.1, ub=T)
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
            m.fix(vx, pos=len(m.time)-1, val=0)
            m.fix(vy, pos=len(m.time)-1, val=0)
            m.fix(vz, pos=len(m.time)-1, val=0)

            # Calculate the distance between the initial and final points
            distance = np.sqrt((Xf-X0)**2 + (Yf-Y0)**2 + (Zf-Z0)**2)
            # Set the speed to reach the final point
            velocity = 0.5

            # Define your scheduled time to reach the final conditions
            tf_scheduled = distance / velocity

            A = 1.0 # Define the weight for prioritizing faster trajectories, maximizing the speed to reach final conditions

            mayers_term = A*(tf-tf_scheduled)**2

            cost = mayers_term #+ desired_position

            # Define the objective
            m.Obj(cost)

            # Solve the optimization problem
            m.options.IMODE = 6
            m.options.SOLVER = 1
            m.solve(disp=False)

            # Print the minimum cost
            print('Minimum Cost: ' + str(m.options.OBJFCNVAL))

            # Print final travel time
            print('Final Time: ' + str(tf.value[0]))

            # Given the X, Y, Z points, interpolate points between each point to get a smooth trajectory
            self.waypoints = [(X[0], Y[0], Z[0])]

            # Make steps a function between the initial and final point 
            steps = min(int(((X0-Xf)**2 + (Y0-Yf)**2 + (Z0-Zf)**2)**0.5)+2, 10)

            for i in range(len(X)-1):
                # Make sure to not include the first point, as it is already included
                x = np.linspace(X[i], X[i+1], steps)[1:]
                y = np.linspace(Y[i], Y[i+1], steps)[1:]
                z = np.linspace(Z[i], Z[i+1], steps)[1:]

                # Append the waypoint only if it is not equal to the previous waypoint
                if (x[0], y[0], z[0]) != self.waypoints[-1]:
                    self.waypoints += [(x[j], y[j], z[j]) for j in range(len(x))]
            print("Number of Waypoints: ", len(self.waypoints))
            
            # Make a 2D plot of the position
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoints))], [v[0] for v in self.waypoints], 'r')
            # ax[0].set_title('X')
            # ax[1].plot([i for i in range(len(self.waypoints))], [v[1] for v in self.waypoints], 'g')
            # ax[1].set_title('Y')
            # ax[2].plot([i for i in range(len(self.waypoints))], [v[2] for v in self.waypoints], 'b')
            # ax[2].set_title('Z')
            # plt.show()

            # Print the waypoints
            # print("Waypoints: ", self.waypoints)

            # Generate the velocity setpoints
            dt = tf_scheduled / len(self.waypoints)
            self.waypoint_velocities = []
            for i in range(len(self.waypoints)-1):
                x = (self.waypoints[i+1][0] - self.waypoints[i][0]) / dt
                y = (self.waypoints[i+1][1] - self.waypoints[i][1]) / dt
                z = (self.waypoints[i+1][2] - self.waypoints[i][2]) / dt
                self.waypoint_velocities.append((x, y, z))
            # Pad the last velocity to match the number of waypoints
            while len(self.waypoint_velocities) < len(self.waypoints):
                # self.waypoint_velocities.append(self.waypoint_velocities[-1])
                self.waypoint_velocities.append((0.0, 0.0, 0.0))
            print("Number of Velocities: ", len(self.waypoint_velocities))

            # Make a 2D plot of the velocities
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoint_velocities))], [v[0] for v in self.waypoint_velocities], 'r')
            # ax[0].set_title('X Velocity')
            # ax[1].plot([i for i in range(len(self.waypoint_velocities))], [v[1] for v in self.waypoint_velocities], 'g')
            # ax[1].set_title('Y Velocity')
            # ax[2].plot([i for i in range(len(self.waypoint_velocities))], [v[2] for v in self.waypoint_velocities], 'b')
            # ax[2].set_title('Z Velocity')
            # plt.show()
                
            # Generate acceleration setpoints
            dt = tf_scheduled / len(self.waypoints)
            self.waypoint_accelerations = []
            for i in range(len(self.waypoint_velocities)-1):
                x = (self.waypoint_velocities[i+1][0] - self.waypoint_velocities[i][0]) / dt
                y = (self.waypoint_velocities[i+1][1] - self.waypoint_velocities[i][1]) / dt
                z = (self.waypoint_velocities[i+1][2] - self.waypoint_velocities[i][2]) / dt
                self.waypoint_accelerations.append((x, y, z))
            # Pad the last acceleration to match the number of waypoints
            while len(self.waypoint_accelerations) < len(self.waypoints):
                # self.waypoint_accelerations.append(self.waypoint_accelerations[-1])
                self.waypoint_accelerations.append((0.0, 0.0, 0.0))
            print("Number of Accelerations: ", len(self.waypoint_accelerations))

            # Make a 2D plot of the accelerations
            # fig, ax = plt.subplots(3, 1, figsize=(10, 10))
            # ax[0].plot([i for i in range(len(self.waypoint_accelerations))], [v[0] for v in self.waypoint_accelerations], 'r')
            # ax[0].set_title('X Acceleration')
            # ax[1].plot([i for i in range(len(self.waypoint_accelerations))], [v[1] for v in self.waypoint_accelerations], 'g')
            # ax[1].set_title('Y Acceleration')
            # ax[2].plot([i for i in range(len(self.waypoint_accelerations))], [v[2] for v in self.waypoint_accelerations], 'b')
            # ax[2].set_title('Z Acceleration')
            # plt.show()


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

        print(f"Current Position: {self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z}")
        print(f"Publishing velocity: {np.round(velocity, 2)}")
        print(f"Publishing acceleration: {np.round(acceleration, 2)}")
        print(f"Publishing yaw: {np.round(yaw, 2)}")
        goal_distance = self.dist2wp(self.x_goal, self.y_goal, self.z_goal)
        print('Goal Distance: ', goal_distance)

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
        """Calculate Euclidean distance to the waypoint"""
        return float((wp_x - self.vehicle_local_position.x)**2 +
                     (wp_y - self.vehicle_local_position.y)**2 + 
                     (wp_z - self.vehicle_local_position.z)**2 )**0.5

    def controller(self) -> None:
        """Callback function to publish drone position"""
        self.publish_offboard_control_heartbeat_signal()
        self.engage_offboard_mode()
        if self.offboard_setpoint_counter  % 20 ==0 :
            current_pos = (self.vehicle_local_position.x ,self.vehicle_local_position.y , self.vehicle_local_position.z)
            goal_pose = (self.x_goal , self.y_goal , self.z_goal)
            goal_distance = self.dist2wp(self.x_goal , self.y_goal  , self.z_goal)

        # Check if the drone is in offboard mode
        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:

            # State machine for the drone
            if self.state == "TAKEOFF":
                # Taking off to a safe altitude before orienting towards goal
                takeoff_height_reached = abs(self.vehicle_local_position.z - self.takeoff_height) < 0.2
                
                if takeoff_height_reached:
                    print("Takeoff complete, switching to ORIENT state")
                    self.state = "ORIENT"
                else:
                    # Set the position setpoint for takeoff (stay in the same XY position)
                    self.waypoint = [self.vehicle_local_position.x, 
                                     self.vehicle_local_position.y, 
                                     self.takeoff_height]
                    
                    # Use a controlled vertical velocity for smooth takeoff
                    self.waypoint_velocity = [0.0, 0.0, -0.5]  # Negative z is up
                    self.waypoint_acceleration = [0.0, 0.0, 0.0]
                    self.yaw_desired = self.vehicle_local_position.heading  # Maintain current heading during takeoff
                    
                    print(f"Taking off to {self.takeoff_height}m, current altitude: {self.vehicle_local_position.z}m")

            elif self.state == "ORIENT":
                # Orient the drone towards the goal
                self.yaw_desired = np.arctan2(self.y_goal - self.vehicle_local_position.y, 
                                             self.x_goal - self.vehicle_local_position.x)
                
                # Set the waypoint to the current position to hover while orienting
                self.waypoint = [self.vehicle_local_position.x, 
                                 self.vehicle_local_position.y, 
                                 self.vehicle_local_position.z]
                
                # Maintain zero velocity and acceleration during orientation
                self.waypoint_velocity = [0.0, 0.0, 0.0]
                self.waypoint_acceleration = [0.0, 0.0, 0.0]
                
                # Check if orientation is complete (within tolerance)
                yaw_error = abs(self.yaw_desired - self.vehicle_local_position.heading)
                yaw_error = min(yaw_error, 2*np.pi - yaw_error)  # Handle wrap-around
                
                if yaw_error < 0.1:  # about 5.7 degrees tolerance
                    print("Orientation complete, switching to READY state")
                    self.state = "READY"
                else:
                    print(f"Orienting towards goal, yaw error: {yaw_error} rad")

            elif self.state == "READY":
                # Ready to start trajectory planning and execution
                if not self.waypoints and not self.goal_reached:
                    # Hold position while waiting for trajectory to be generated
                    self.waypoint = [float(self.vehicle_local_position.x), 
                                     float(self.vehicle_local_position.y), 
                                     float(self.vehicle_local_position.z)]
                    self.waypoint_velocity = [0.0, 0.0, 0.0]
                    self.waypoint_acceleration = [0.0, 0.0, 0.0]
                    print("Ready and waiting for trajectory generation...")
                else:
                    # Trajectory generated, switch to waypoint navigation
                    self.state = "WAYPOINT_NAV"
                    print("Trajectory generated, switching to WAYPOINT_NAV state")

            elif self.state == "WAYPOINT_NAV":
                
                # Execute waypoint navigation
                self.navigation_timeout += 1 
                if self.navigation_timeout > self.max_navigation_time:
                    print("Navigation timeout reached ,checking iof close to goal")
                    goal_distance = self.dist2wp(self.x_goal , self.y_goal , self.z_goal)
                    if goal_distance < 2.0:
                        self.waypoint = [float(self.vehicle_local_position.x),
                                         float(self.vehicle_local_position.y),
                                         float(self.vehicle_local_position.z)]
                        self.waypoint_counter = 1 
                        self.state = "GOAL_REACHED"
                        self.goal_reached = True
                    else:
                        self.waypoint = [float(self.vehicle_local_position.x),
                                         float(self.vehicle_local_position.y),
                                         float(self.vehicle_local_position.z)]
                        self.state = "READY"
                    self.navigation_timeout = 0
                if self.waypoints:
                    # Calculate the distance to the current waypoint
                    distance_to_waypoint = self.dist2wp(float(self.waypoints[self.waypoint_counter][0]), 
                                                        float(self.waypoints[self.waypoint_counter][1]), 
                                                        float(self.waypoints[self.waypoint_counter][2]))

                    # Check if the drone is close to the waypoint
                    if distance_to_waypoint < 0.5:
                        self.waypoint_counter += 1
                    
                    # Limit the waypoint counter
                    self.waypoint_counter = min(self.waypoint_counter, len(self.waypoints)-1)

                    # Calculate the desired yaw, facing the next waypoint
                    self.yaw_desired = np.arctan2(self.waypoints[self.waypoint_counter][1] - self.waypoints[self.waypoint_counter-1][1], 
                                                 self.waypoints[self.waypoint_counter][0] - self.waypoints[self.waypoint_counter-1][0])
                    # Set the waypoint position
                    self.waypoint = [float(self.waypoints[self.waypoint_counter][0]),
                                    float(self.waypoints[self.waypoint_counter][1]),
                                    float(self.waypoints[self.waypoint_counter][2])]
                    
                    # Set the waypoint velocity
                    self.waypoint_velocity = [float(self.waypoint_velocities[self.waypoint_counter][0]),
                                             float(self.waypoint_velocities[self.waypoint_counter][1]), 
                                             float(self.waypoint_velocities[self.waypoint_counter][2])]
                    
                    # Set the waypoint acceleration
                    self.waypoint_acceleration = [float(self.waypoint_accelerations[self.waypoint_counter][0]),
                                                 float(self.waypoint_accelerations[self.waypoint_counter][1]), 
                                                 float(self.waypoint_accelerations[self.waypoint_counter][2])]

                    # Print distance to current waypoint
                    print('Distance to waypoint: ', distance_to_waypoint)
                    print(f'Waypoint {self.waypoint_counter} reached out of {len(self.waypoints)-1} waypoints')

                    # If at the final waypoint, set the waypoints to False
                    if self.waypoint_counter == len(self.waypoints)-1:
                        self.waypoints = False
                        self.waypoint_counter = 1
                        self.state = "GOAL_REACHED"
                        self.goal_reached = True
                        print('All waypoints reached, mission completed')
                    goal_distance = self.dist2wp(self.x_goal , self.y_goal , self.z_goal)
                    if goal_distance <1.0:
                        self.waypoint =  [float(self.waypoint_accelerations[self.waypoint_counter][0]),
                                                 float(self.waypoint_accelerations[self.waypoint_counter][1]), 
                                                 float(self.waypoint_accelerations[self.waypoint_counter][2])]
                        self.waypoint_counter = 1 
                        self.state = "GOAL_REACHED"
                        self.goal_reached = True
                        print("Goal reached directly , skipping remaining waypoints")
                else:
                    # No waypoints available, reset to READY state to regenerate trajectory
                    self.state = "READY"

            elif self.state == "GOAL_REACHED":
                # Hold position at the goal
                self.goal_wait_counter += 1

                # Hold position
                self.waypoint = [float(self.vehicle_local_position.x), 
                                float(self.vehicle_local_position.y), 
                                float(self.vehicle_local_position.z)]
                self.waypoint_velocity = [0.0, 0.0, 0.0]
                self.waypoint_acceleration = [0.0, 0.0, 0.0]

                # If the goal wait counter is greater than the wait time, prepare for next goal
                if self.goal_wait_counter > self.wait_time:
                    self.goal_wait_counter = 0
                    self.goal_reached = False

                    # Increment and loop the goal counter
                    self.goal_counter += 1
                    self.goal_counter = self.goal_counter % len(self.goal_points)

                    # Set the new goal position
                    self.x_goal, self.y_goal, self.z_goal = self.goal_points[self.goal_counter]
                    
                    # Switch back to ORIENT state to face the new goal
                    self.state = "ORIENT"
                    print('New goal position: ', self.x_goal, self.y_goal, self.z_goal)

            # Print the current state and variables
            print(f'Current state: {self.state}')
            print('Current goal: ', self.x_goal, self.y_goal, self.z_goal)
            print('Goal Reached: ', self.goal_reached)
            print('Waypoints available: ', 'Yes' if self.waypoints else 'No')

            # Publish the position setpoint
            self.publish_position_setpoint(self.waypoint, self.waypoint_velocity,
                                          self.waypoint_acceleration, self.yaw_desired)
            print("\n")
def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)