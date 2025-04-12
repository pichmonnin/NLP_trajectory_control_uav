from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="navi",
            executable="Joy_control",
            name="joystick_drone_control",
            output = "screen"
        ),
        Node(
            package='joy_linux',
            executable="joy_linux_node",
            name="joy_node",                                                                                                                
            output="screen"
        )
    ])                                                                                                                      