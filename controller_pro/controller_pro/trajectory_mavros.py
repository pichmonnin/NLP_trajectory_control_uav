#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped , Transform , Twist , Vector3 , Quaternion 
from trajectory_msgs.msg import MultiDOFJointTrajectory ,MultiDOFJointTrajectoryPoint
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile ,QoSHistoryPolicy , QoSDurabilityPolicy , QoSReliabilityPolicy
from gekko import GEKKO
from tf2_ros import TransformBroadcaster , TransformStamped
import time
import math
import numpy as np

class DroneControl(Node):
    def __init__(self):
        super().__init__('fly_up_node')
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5
            
        )
        self.pos_pub = self.create_publisher(PoseStamped , "/mavros/setpoint_position/local" ,10)
        self.traj_pub  = self.create_publisher(MultiDOFJointTrajectory ,"/mavros/setpoint_trajectory/local" ,10)
        self.arm_client = self.create_client(CommandBool , "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        
        self.state_sub = self.create_subscription(State , "/mavros/state" ,self.state_cb , 10)
        
        self.local_pose = PoseStamped()
        self.local_pose_received = False
        self.local_pose_sub = self.create_subscription(PoseStamped , "/mavros/local_position/pose" , self.local_pose_cb ,qos_profile)
        
        self.current_state =State()
        self.state_received = False
        
        self.quaternion = Quaternion()
        
        #Initial variable for trajectory control
        
        self.dt = 0.05
        self.wait_time = int(5/self.dt)
        self.goal_points = [(5.0 ,2.0 , 3.0),(10.0 ,5.0 , 5.0) , (2.0 ,3.0 ,6.0) , (7.0 , 5.0 , 7.0)]
        self.goal_counter = 0 
        self.goal_wait_counter = 0 
        self.goal_reached = False
        self.x_goal , self.y_goal , self.z_goal =self.goal_points[self.goal_counter]
        self.navigation_timeout = 0
        self.max_navigation_time = int(20/self.dt)
        
        #Waypoint navigation generations
        self.waypoints = []
        self.waypoints_counter = 0 
        self.waypoints_velocities = []
        self.waypoints_accelerations = []
        # Waypoint navigation current 
        self.waypoint = [0.0 , 0.0,0.0]
        self.waypoint_velocity = [0.0,0.0,0.0]
        self.waypoint_acceleration = [0.0,0.0,0.0]
        self.yaw_desired = 0.0 
        
        self.pos_msg = PoseStamped()
        self.pos_msg.pose.position.x = 0.0
        self.pos_msg.pose.position.y = 0.0
        self.pos_msg.pose.position.z = 0.0
        self.pos_msg.pose.orientation.w = 1.0
        self.last_req_time =self.get_clock().now()
        self.control_stages ={
            'init' :True,
            'pre_offboard' :False , 
            'set_offboard' :False ,
            'arming' : False , 
            'flying' : False, 
            'complete' : False,
            'orient' : False,
            'waypoint_navigation' : False ,
            'setpoints_reached' : False ,
            'hovering' : False,
        }
        self.target_height = 2.0
        
        self.timer  =self.create_timer(0.1 , self.control_loop)
        self.start_time =self.get_clock().now()
        self.get_logger().info("FlyNode initialized")
        
        
    def state_cb(self,msg):
        self.current_state = msg 
        self.state_received = True
        
    def local_pose_cb(self , msg):
        self.local_pose = msg
        self.local_pose_received = True
        
        if self.control_stages['init']:
            self.pos_msg.pose.position.x = msg.pose.position.x
            self.pos_msg.pose.position.y = msg.pose.position.y
            self.pos_msg.pose.position.z = msg.pose.position.z
            self.pos_msg.pose.orientation = msg.pose.orientation
    def control_loop(self):
        self.pos_msg.header.stamp = self.get_clock().now().to_msg()
        self.pos_msg.header.frame_id = 'map'
        
        self.pos_pub.publish(self.pos_msg)
        if not self.state_received or not self.local_pose_received:
            return
        if self.control_stages['init']:
            if self.current_state.connected:
                self.get_logger().info("FCU Connected")
                self.control_stages['init'] =False
                self.control_stages['pre_offboard'] = True
                
                self.start_time =self.get_clock().now()
            return
        if self.control_stages['pre_offboard']:
            elaspsed = (self.get_clock().now() - self.start_time).nanoseconds /1e9
            if elaspsed > 2.0:
                self.control_stages['pre_offboard'] = False
                self.control_stages['set_offboard'] = True
                self.get_logger().info("Pre-flight setpoints send for 2 second")
        if self.control_stages['set_offboard']:
            if self.current_state.mode != "OFFBOARD":
                if self.check_request_interval():
                    self.set_mode("OFFBOARD")
            else:
                self.get_logger().info("Offboard mode enabled")
                self.control_stages['set_offboard'] = False
                self.control_stages['arming'] = True
            return
        if self.control_stages['arming']:
            if not self.current_state.armed:
                if self.check_request_interval():
                    self.arm_drone()
            else:
                self.get_logger().info("Drone is armed")
                self.control_stages['arming'] = False
                self.control_stages['flying'] = True
                self.start_time = self.get_clock().now()
                    
                self.pos_msg.pose.position.z = self.local_pose.pose.position.z + self.target_height
            return
            
        if self.control_stages['flying']:
            current_height = self.local_pose.pose.position.z 
            target_height = self.pos_msg.pose.position.z
            height_error = abs(target_height - current_height)
            if height_error <0.1:
                self.get_logger().info(f"Target_reached {current_height:.2f}m .hoveset_offboardring")
                self.control_stages['flying'] = False
                self.control_stages['complete'] = True
            else:
                elaspsed = (self.get_clock().now() - self.start_time).nanoseconds /1e9
                if elaspsed > 10.0:
                    self.get_logger().warn(f"Timeout Reached , current height: {current_height:.2f}m")
                    self.control_stages['flying'] = False
                    self.control_stages['complete'] = True
                else:
                    
                    self.get_logger().info(f"Ascending : Current {current_height:.2f}m , Target_height{target_height:.2f}m , {height_error:.2f} m ")
        if self.control_stages["complete"]:
            self.control_stages['complete'] = False
            self.control_stages['orient'] = True
        if self.control_stages["orient"]:
            self.handle_orientation_stage()
        if self.control_stages['waypoint_navigation']:
            self.execute_waypoint_navigation()
        if self.control_stages['setpoints_reached']:
            self.handle_drone_hovring()
        if self.control_stages['hovering']:
            self.pos_msg.pose.position.x = self.hover_position[0]
            self.pos_msg.pose.position.y = self.hover_position[1]
            self.pos_msg.pose.position.z = self.hover_position[2]
            self.pos_msg.pose.orientation = self.yaw_to_quaternion(self.hover_yaw)
            self.pos_pub.publish(self.pos_msg)
            if self.check_remaining_goals():
                self.control_stages['hovering'] = False
                self.control_stages['orient'] = True
            

        
    def arm_drone(self):
        arm_req = CommandBool.Request()
        arm_req.value = True
        self.get_logger().info("Attempting to arm")
        future = self.arm_client.call_async(arm_req)
        future.add_done_callback(self.arm_callback) 
    
    
    
    def arm_callback(self ,future):
        try:
            response = future.result()
            if response.success():
                self.get_logger().info("Arming is Success")
            else:
                self.get_logger().info("Arming Failed")
        except Exception as e:
            self.get_logger().warn(f"Service called failed:{e}")
    
    def set_mode(self, mode):
        mode_req = SetMode.Request()
        mode_req.custom_mode = mode
        self.get_logger().info(f"Attempting to set {mode} mode ....")
        future = self.mode_client.call_async(mode_req)
        future.add_done_callback(self.mode_callback)
    
    def mode_callback(self  , future):
        try:
            response  = future.result()
            if response.mode_sent:
                self.get_logger().info("Info mode changed request sent")
            else:
                self.get_logger().info("Mode changed request failed")
        except Exception as e :
                self.get_logger().error(f"Service call failed: {e}")
    
    def check_request_interval(self):
        current_time = self.get_clock().now()
        if (current_time - self.last_req_time).nanoseconds/ 1e9 > 1.0:
            self.last_req_time = current_time
            return True 
        
    def yaw_to_quaternion(self , yaw):
        self.quaternion.x = 0.0
        self.quaternion.y = 0.0
        self.quaternion.z = math.sin(yaw /2)
        self.quaternion.w = math.cos(yaw/2)
        return self.quaternion
        
    
    def trajectory_generator(self):
        if not self.waypoints:
            try:
                X0 = float(self.local_pose.pose.position.x)
                Y0 = float(self.local_pose.pose.position.y)
                Z0 = float(self.local_pose.pose.position.z)
                vx0 , vy0 , vz0 = 0.0 , 0.0,0.0
                
                self.get_logger().info(f"Optimizer Starting position:{X0} , {Y0} , {Z0}")
                
                Xf = float(self.x_goal)
                Yf = float(self.y_goal)
                Zf = float(self.z_goal)
                
                v_max , v_min = 3.0 , -3.0
                a_max , a_min = 1.0 , -1.0
                
                m = GEKKO(remote= False)
                steps = min(int(((X0-Xf)**2 +(Y0-Yf)**2 +(Z0-Zf)**2)**0.5)+5 , 10)
                T = 5.0
                m.time = np.linspace(0 , T,steps)
                
                X = m.Var(value=X0)
                Y = m.Var(value=Y0)
                Z = m.Var(value=Z0)
                vx = m.Var(value=vx0,lb = v_min  ,ub=v_max)
                vy = m.Var(value=vy0,lb = v_min  ,ub=v_max)
                vz = m.Var(value=vz0,lb = v_min  ,ub=v_max)
                ax = m.MV(value=0.0,lb = a_min  ,ub=a_max)
                ay = m.MV(value=0.0,lb = a_min  ,ub=a_max)
                az = m.MV(value=0.0,lb = a_min  ,ub=a_max)

                ax.STATUS = 1
                ay.STATUS = 1 
                az.STATUS = 1
                
                
                tf = m.FV(value=1.0 , lb=0.1 ,ub=T)
                tf.STATUS = 1
                
                m.Equation(X.dt() == vx*tf) 
                m.Equation(Y.dt() == vy*tf)
                m.Equation(Z.dt() == vz*tf)
                
                m.Equation(vx.dt() == ax*tf) 
                m.Equation(vy.dt() == ay*tf)
                m.Equation(vz.dt() == az*tf)

                m.fix(X , pos=len(m.time)-1 , val=Xf)
                m.fix(Y , pos=len(m.time)-1 , val=Yf)
                m.fix(Z , pos=len(m.time)-1 , val=Zf)
                m.fix(vx , pos=len(m.time)-1 , val=0.0)
                m.fix(vy, pos=len(m.time)-1 , val=0.0)
                m.fix(vz , pos=len(m.time)-1 , val=0.0)
                
                distance = np.sqrt((Xf-X0)**2 +(Yf-Y0)**2+(Zf-Z0)**2)      
                velocity = 0.5
                tf_scheduled = distance/velocity
                
                A=1.0
                mayers_term  = A*(tf-tf_scheduled)**2
                cost = mayers_term
                
                m.Obj(cost)
                m.options.IMODE = 6 
                m.options.SOLVER = 1
                m.solve(disp=False)                  
                self.get_logger().info(f"Minimum Cost:{m.options.OBJFCNVAL}")
                self.get_logger().info(f"Final time: {tf.value[0]}")
                
                self.waypoints = [(X[0] , Y[0], Z[0])]
                steps = min(int(distance)+2 ,10)
                
                for i in range(len(X)-1):
                    x = np.linspace(X[i] , X[i+1] ,steps)[1:]
                    y = np.linspace(Y[i] , Y[i+1] ,steps)[1:]
                    z = np.linspace(Z[i] , Z[i+1] ,steps)[1:]
                    if (x[0] , y[0],z[0]) != self.waypoints[-1]:
                        self.waypoints += [(x[j] , y[j],z[j]) for j in range(len(x))]
                self.get_logger().info(f"Number of waypoints : {len(self.waypoints)}")
                dt = tf_scheduled / (len(self.waypoints)-1)
            
            
                self.waypoints_velocities = []
                for i in range(len(self.waypoints) -1 ):
                    x = (self.waypoints[i+1][0] -self.waypoints[i][0]) /dt 
                    y = (self.waypoints[i+1][1] -self.waypoints[i][1]) /dt
                    z = (self.waypoints[i+1][2] -self.waypoints[i][2]) /dt
                    self.waypoints_velocities.append((x,y,z))
                
                self.waypoints_velocities.append((0.0,0.0,0.0))
                self.get_logger().info(f"Number of Velocities:{len(self.waypoints_velocities)}")
                
                self.waypoints_accelerations=[]
                for i in range(len(self.waypoints_velocities)-1):
                    x = (self.waypoints_velocities[i+1][0] -self.waypoints_velocities[i][0])
                    y = (self.waypoints_velocities[i+1][1] -self.waypoints_velocities[i][1])
                    z = (self.waypoints_velocities[i+1][2] -self.waypoints_velocities[i][2])      
                    self.waypoints_accelerations.append((x,y,z))
                
                self.waypoints_accelerations.append((0.0,0.0,0.0))
                self.get_logger().info(f"Number of Accelerations :{len(self.waypoints_accelerations)}")
            except Exception as e :
                self.get_logger().error(f"Trajectory generation failed:{str(e)}")
                self.waypoints =[]
            
            if self.waypoints:
                self.get_logger().info(f"First few waypoints")
                for i in range(min(3 , len(self.waypoints))):
                    self.get_logger().info(f"Waypoint {i}: {self.waypoints[i]}")
                    if i < len(self.waypoints_velocities):
                        self.get_logger().info(f' Velocity {i} :{self.waypoints_velocities[i]}')        

    def dist2wp(self , wp_x  , wp_y,wp_z)-> float:
        return float((wp_x-self.local_pose.pose.position.x)**2+
                     (wp_y-self.local_pose.pose.position.y)**2+
                     (wp_z-self.local_pose.pose.position.z)**2)**0.5
    
    def handle_orientation_stage(self):
        dx = self.x_goal - self.local_pose.pose.position.x
        dy = self.y_goal - self.local_pose.pose.position.y
        target_yaw = math.atan2(dy , dx)
        current_yaw =self.get_current_yaw()
        self.yaw_error = abs(target_yaw - current_yaw)
        
        if self.yaw_error < 0.1:
            self.get_logger().info("Already oriented .Skipping re orientation")
            self.control_stages['orient'] = False
            self.yaw_desired = target_yaw
            self.handle_waypoint_init()
            return
        self.pos_msg.pose.orientation = self.yaw_to_quaternion(target_yaw)
        self.get_logger().info(f"Orienting: Current={current_yaw:.2f}, Target={target_yaw:.2f} , Error ={self.yaw_error:.2f}")
        if self.yaw_error < 0.1:
            self.get_logger().info("Orientation Complete")
            self.control_stages['orient'] = False
            self.yaw_desired = target_yaw
            self.handle_waypoint_init()
    
    def handle_waypoint_init(self):
        if  not self.waypoints :
            self.get_logger().info("Generating trajectory waypoints  to goal point")
            self.trajectory_generator()
            
            if self.waypoints:
                self.control_stages["waypoint_navigation"] = True
                self.waypoints_counter = 0
                self.start_time = self.get_clock().now()
                self.get_logger().info(f"Starting  navigation with {len(self.waypoints)} waypoints")
            else:
                self.get_logger().error("Failed to generate waypoints")
                self.control_stages['complete'] = True

    

    
    
    def execute_waypoint_navigation(self):
        self.get_logger().info(f"Executing waypoint {self.waypoints_counter}/{len(self.waypoints)}")
        if self.waypoints_counter >= len(self.waypoints):
            if not self.goal_reached:
                self.get_logger().info("Reached final waypoint!")
                self.control_stages['setpoints_reached'] = True
                self.goal_reached = True
            return
        wp = self.waypoints[self.waypoints_counter]
        vel= [0.0,0.0,0.0]
        acc =[0.0,0.0,0.0]
        if self.waypoints_counter < len(self.waypoints_velocities):    
            vel = self.waypoints_velocities[self.waypoints_counter]
        if self.waypoints_counter < len(self.waypoints_accelerations):
            acc  = self.waypoints_accelerations[self.waypoints_counter]
            
        self.publish_position_setpoint(wp , vel ,acc , self.yaw_desired)
        
        dist = self.dist2wp(wp[0],wp[1],wp[2])
        threshold = 0.2
        if dist < threshold:
            self.waypoints_counter += 1 
            self.get_logger().info(f"Waypoint{self.waypoints_counter}/{len(self.waypoints)}:"
                                   f"distance{dist:.2f} m , goal_distance{self.dist2wp(self.x_goal , self.y_goal , self.z_goal):.2f} m")
            
    def get_current_yaw(self):
        q = self.local_pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y**2 + q.z**2))                   
            

    def handle_drone_hovring(self):
        if self.control_stages['setpoints_reached'] ==True:
            self.hover_position=[
                self.local_pose.pose.position.x,
                self.local_pose.pose.position.y,
                self.local_pose.pose.position.z
            ]
            self.hover_yaw = self.get_current_yaw()
            self.control_stages['hovering'] = True
            self.control_stages['setpoints_reached'] = False
            self.control_stages['waypoint_navigation'] = False
            
    def check_remaining_goals(self):
        if self.goal_counter +1 < len(self.goal_points):
            self.goal_counter += 1 
            self.x_goal , self.y_goal , self.z_goal = self.goal_points[self.goal_counter]
            self.control_stages['hovering'] = False
            self.control_stages['orient'] = True
            self.control_stages['waypoint_navigation'] = False
            self.waypoints =[]
            self.waypoints_counter = 0 
            self.goal_reached = False
            self.get_logger().info(f"Next goal set:{self.x_goal} , {self.y_goal} ,{self.z_goal}")
            return True
        else:
            self.get_logger().info("All goal reached. Staying in hover mode")
            return False
                
    def publish_position_setpoint(self, position: list , velocity: list  , acceleration: list  , yaw:float):
        current_pos = (self.local_pose.pose.position.x , self.local_pose.pose.position.y , self.local_pose.pose.position.z)
        goal_distance = self.dist2wp(self.x_goal , self.y_goal , self.z_goal)
        self.get_logger().info(f"Goal Distance :{goal_distance}")
        msg = MultiDOFJointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        point = MultiDOFJointTrajectoryPoint()
        point.transforms = [Transform()]
        point.transforms[0].translation.x = position[0]
        point.transforms[0].translation.y = position[1]
        point.transforms[0].translation.z = position[2]
        
        point.velocities = [Twist()]
        point.velocities[0].linear.x  = velocity[0]
        point.velocities[0].linear.y  = velocity[1]
        point.velocities[0].linear.z  = velocity[2]
                    
        point.accelerations = [Twist()]
        point.accelerations[0].linear.x = acceleration[0]
        point.accelerations[0].linear.y = acceleration[1]
        point.accelerations[0].linear.z = acceleration[2]
        
        quat = self.yaw_to_quaternion(yaw)
        point.transforms[0].rotation = quat
        
        msg.points.append(point)
        self.traj_pub.publish(msg)
        self.get_logger().debug(f"Published setpoint: pos={position}, vel={velocity}")


def main(args=None):
    rclpy.init(args=args)
    node = DroneControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()