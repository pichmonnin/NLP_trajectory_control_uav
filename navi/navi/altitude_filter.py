import rclpy 
from rclpy.node import Node
from px4_msgs.msg import SensorCombined , VehicleOdometry# type: ignore
from sensor_msgs.msg import Imu
import numpy as np 
from geometry_msgs.msg import Point , Pose
from rclpy.qos import QoSProfile , QoSReliabilityPolicy , QoSHistoryPolicy  , QoSDurabilityPolicy
from std_msgs.msg import Float32
import matplotlib.pyplot as plt 
from collections import deque
import threading
import time 

class EKFAltitude(Node):
    def __init__(self):
        super().__init__("ekf_altitude_filter")
        qos_profile = QoSProfile(
            reliability= QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history = QoSHistoryPolicy.KEEP_LAST, 
            depth = 5
        )
        #Subcriber 
        self.IMU_subscription = self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined" ,
            self.imu_callback,
            qos_profile
        )
        self.Odom_subscription = self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.odom_callback,
            qos_profile
            
        )
        
        #Publisher 
        self.Pose_pub = self.create_publisher(
            Pose ,
            "/ekf_pose",
            qos_profile
        )
        self.dt =0.02
        self.gravity = 9.81
        
        self.x = np.zeros((12,1))
        self.P = np.eye(12)*1000
        
        self.F = np.eye(12)
        self.H = np.eye(12)
        self.Q = np.eye(12) *0.1
        self.R = np.eye(12)*0.5      
         
    def imu_callback(self, msg:SensorCombined):
        raw_acc = np.array([[msg.accelerometer_m_s2[0]],
                            [msg.accelerometer_m_s2[1]],
                            [msg.accelerometer_m_s2[2]]])
        raw_gyro = np.array([[msg.gyro_rad[0]],
                             [msg.gyro_rad[1]],
                             [msg.gyro_rad[2]]])
        self.x[0:3] += self.x[3:6] * self.dt
        self.x[3:6] += (raw_acc - np.array([[0] ,[0] ,[self.gravity]])) *self.dt
        self.x[6:9] += raw_gyro *self.dt
        
        self.P = np.dot(self.F , np.dot(self.P ,self.F.T)) + self.Q
        pose_msg = Pose()
        pose_msg.position.x = float(self.x[0,0])
        pose_msg.position.y = float(self.x[1,0])
        pose_msg.position.z = float(self.x[2,0])
        
        q = self.euler_to_quaternion(self.x[6,0] ,self.x[7,0] ,self.x[8,0])
        pose_msg.orientation.x = float(q[0])
        pose_msg.orientation.y =float(q[1])
        pose_msg.orientation.z = float(q[2])
        pose_msg.orientation.w = float(q[3])
        
        self.Pose_pub.publish(pose_msg)
    def odom_callback(self, msg: VehicleOdometry):
        measurement = np.array([[msg.velocity[0]],
                                [msg.velocity[1]],
                                [msg.velocity[2]]])
        y = measurement -self.x[3:6]
        S =self.P[3:6,3:6] +self.R[3:6 ,3:6]
        K = self.P[3:6, 3:6] @ np.linalg.inv(S)
        self.x[3:6] += np.dot(K ,y)
        self.P[3:6 ,3:6] -= K @ self.P[3:6,3:6]
    def euler_to_quaternion(self, roll ,pitch , yaw):
        cy = np.cos(yaw* 0.5)
        sy = np.sin(yaw *0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll *0.5)
        sr = np.sin(roll *0.5)
        
        q = np.zeros(4)
        q[0]=  sr *cp * cy -cr *sp * sy
        q[1]= cr* sp *cy + sr * cp *sy
        q[2] = cr * cp * sy - sr * sp * cy
        q[3] = cr * cp * cy + sr * sp * sy
        return q 
        
     
        
        
        

def main(): 
    rclpy.init()
    node = EKFAltitude()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == "__main__":
    main()

