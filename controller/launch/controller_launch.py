import launch
import launch_ros.actions
import launch.action

def generate_launch_description():
    return launch.LaunchDescription([
       launch_ros.actions.Node(
            package="joy_linux", 
            executable="joy_linux_node",
            name="joy_node",
            output ="screen"
        ),       
        launch.actions.TimerAction(
            period =5.0,
            actions=[
                launch_ros.actions.Node(
                    package="controller",
                    executable="ekf_filter",
                    name = "ekf_altitude_filter",
                    output = "screen"
                )
            ]
        ),
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                launch_ros.actions.Node(
                    package="controller", 
                    executable="Joy_stick_control",
                    name= "Joy_drone_control",
                    output="screen"
                )
            ]
        )
        
    ])