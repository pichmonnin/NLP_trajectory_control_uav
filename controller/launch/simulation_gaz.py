import launch
import launch_ros.actions 
from launch.actions import ExecuteProcess , TimerAction , GroupAction 
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        GroupAction([
            TimerAction(
                period= 5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["MicroXRCEAgent", "udp4","-p","8888"],
                        output="screen"
                    )
                ]
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["make","px4_sitl","gz_x500_lidar_2d"],
                        cwd="/home/pich/PX4-Autopilot",
                        output="log"
                    )
                ]
            )
        ])
    ])