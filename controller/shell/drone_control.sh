#!/bin/bash

function usage() {
  echo "Usage: $0 {arm|disarm|land|goto x y z}"
  echo "  arm         : Arm the drone"
  echo "  disarm      : Disarm the drone"
  echo "  land        : Land the drone"
  echo "  goto x y z  : Send a manual setpoint (in meters)"
  exit 1
}

case "$1" in
  arm)
    ros2 service call /arm_drone std_srvs/srv/SetBool "{data: true}"
    ;;
  disarm)
    ros2 service call /arm_drone std_srvs/srv/SetBool "{data: false}"
    ;;
  land)
    ros2 service call /land_drone std_srvs/srv/Trigger "{}"
    ;;
  goto)
    if [ $# -ne 4 ]; then
      echo "Error: goto requires 3 arguments (x y z)"
      usage
    fi
    ros2 topic pub --once /manual_setpoint geometry_msgs/msg/Point "{x: $2, y: $3, z: $4}"
    ;;
  *)
    usage
    ;;
esac
