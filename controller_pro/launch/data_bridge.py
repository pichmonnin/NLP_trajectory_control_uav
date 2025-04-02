from launch import LaunchDescription 
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/world/default/model/x500_flow_0/link/flow_link/sensor/flow_camera/image@sensor_msgs/msg/Image@ignition.msgs.Image",
                "/world/default/model/x500_flow_0/link/lidar_sensor_link/sensor/lidar/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
                "/world/default/model/x500_flow_0/link/lidar_sensor_link/sensor/lidar/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            ]
        ),
        Node(
            package="tf2_ros", 
            executable="static_transform_publisher",
            arguments=["0.1" , "0",'0.05', '0' ,'0','0','base_link','x500_flow_0/lidar_sensor_link/lidar']
            
        )
    ])