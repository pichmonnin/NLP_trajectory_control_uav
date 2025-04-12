import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile , ReliabilityPolicy  , HistoryPolicy , DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray

class PathVisualizer(Node):
    def __init__(self):
        super().__init__("drone_visualizer")
        qos_profile = QoSProfile(
            reliability =ReliabilityPolicy.BEST_EFFORT,
            durability =DurabilityPolicy.TRANSIENT_LOCAL,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.subscription = self.create_subscription(
            Path,
            "/drone_path_plan",
            self.path_callback,
            qos_profile
        )
        self.marker_pub = self.create_publisher(Marker , "/vis_marker" , qos_profile)
        self.create_timer(0.1  , self.publish_drone_marker)
        self.current_position = Point(x=0.0 , y=0.0 , z=0.0)
    def path_callback(self , msg):
        if not msg.poses:
            return
        marker = Marker()
        marker.header = msg.header
        marker.ns = "drone_path"
        marker.id = 0 
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a =1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        for pose in msg.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z
            marker.points.append(point)
        self.marker_pub.publish(marker) 
    
    
    
    def visualize_path(self):
        if not self.current_path:
            marker = Marker()
            marker.header = self.current_path
            marker.ns = "path"
            marker.id = 0 
            marker.type =Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1 
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g =1.0
            marker.color.b =0.0
            
            for pose in self.current_path.poses:
                point = Point()
                point.x = pose.pose.position.x
                point.y = pose.pose.position.y
                point.z = pose.pose.position.z
                marker.points.append(point)
            self.marker_pub.publish(marker)
            
    def publish_drone_marker(self):
        marker= Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone"
        marker.id = 1 
        marker.type = Marker.SPHERE
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0 
        marker.pose.position.z = 0.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a =1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        
        self.marker_pub.publish(marker)
        
def main(args = None):
    rclpy.init(args=args)
    visualizer = PathVisualizer()
    rclpy.spin(visualizer)
    visualizer.destroy_node()
    rclpy.shutdown()
if __name__ == "main":
    main()
        
