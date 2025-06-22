# Implementation of Reinforcement learning algorithms for local PathPlanning and NLP for Global Path Planning 🚀

## 📌 Prerequisites

### **Required Software**

- **ROS 2 Humble (Ubuntu 22.04)**
- **Mavros for Humble**
- **PX4 Autopilot**
---
    


## 📦 Dependencies for ROS 2 Humble
    pip3 install --user -U empy pyros-genmsg setuptools
    pip3 install kconfiglib
    pip install --user jsonschema
    pip install --user jinja2
---

## 🏗 Installation Steps
### ** Step 1 : Install Mavros**
    sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
---

### **  Step 2: GeographicLib Dataset**
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh

### ** Step 3 : Install PX4 Autopilot**
    git clone https://github.com/PX4/px4_msgs.git -b -recursive release/1.15
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    Sudo reboot  
**Note:** You have to reboot your PC for this to take effect

**Note:** When you execute ubuntu.sh there would be an error so what you have to do is to go to that directory and inside the requirement.txt file you have to change matplot 3.0.* to 3.0 instead. After that you need the bash ./PX4-Autopilot/Tools/setup/ubuntu.sh command again


### ** Step 4 : Install the ros2_bridge**
    sudo apt install ros-humble-ros-gzgarden
---
- Always **source** your workspace after building.
- Double-check **dependencies** if you encounter errors.
- Follow **specific package versions** to prevent build issues.
🚀 Running the Project

Once all prerequisites are installed, you can launch different components of the project using the following commands but first you have to clone the repository:
### **Step 2: Clone the Repository **
    git clone https://github.com/pichmonnin/NLP_trajectory_control_uav.git
### **Step 2: run the NLP Algorithm **
    ros2 run controller_pro trajectory_mavros.py
### **Step 3: bridge PX4 with mavros **
    ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
---
This is how we can establish connection between the gazebo simulation and our ros2 node

### **Step 4: bridge gazebo data with ROS2**
    ros2 launch controller_pro camera_bridge.py









