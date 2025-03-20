from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gazebo_ros2_bridge", 
            arguments=[
                "/world/gamefield/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"
            ],
            output="screen"
        ),
        Node(
            package="tf2_ros", 
            executable="static_transform_publisher", 
            arguments=["0","0","1","0","0","0","map","x500_lidar_2d_0/link/lidar_2d_v2"],
            name="static_tf"
        ),
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="rviz2",
                    output="screen",
                )
            ]
        )
    ])