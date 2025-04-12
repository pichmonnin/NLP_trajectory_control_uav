#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from rclpy.qos import QoSProfile ,QoSHistoryPolicy , QoSDurabilityPolicy , QoSReliabilityPolicy
import time

class FlyUpNode(Node):
    def __init__(self):
        super().__init__('fly_up_node')
        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 5
            
        )
        self.pos_pub = self.create_publisher(PoseStamped , "/mavros/setpoint_position/local" ,10)
        
        self.arm_client = self.create_client(CommandBool , "/mavros/cmd/arming")
        self.mode_client = self.create_client(SetMode, "/mavros/set_mode")
        
        self.state_sub = self.create_subscription(State , "/mavros/state" ,self.state_cb , 10)
        
        self.local_pose = PoseStamped()
        self.local_pose_received = False
        self.local_pose_sub = self.create_subscription(PoseStamped , "/mavros/local_position/pose" , self.local_pose_cb ,qos_profile)
        
        self.current_state =State()
        self.state_received = False
        
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
            'complete' : False
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

def main(args=None):
    rclpy.init(args=args)
    node = FlyUpNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()