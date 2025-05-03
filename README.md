# Implementation of Reinforcement learning algorithms for local PathPlanning and NLP for Global Path Planning 🚀

## 📌 Prerequisites

### **Required Software**

- **ROS 2 Humble (Ubuntu 22.04)**\
  ⚠ **Note:** Setup Tool must be **version 65.5.1** to prevent build errors with `px4_msgs`.
- **Mavros for Humble**
- **PX4 Autopilot**
- **Gazebo Igition**


---
    


## 📦 Dependencies for ROS 2 Humble
    pip3 install --user -U empy pyros-genmsg setuptools
    pip3 install kconfiglib
    pip install --user jsonschema
    pip install --user jinja2

---

## 🏗 Installation Steps

### **1️⃣ Install Gazebo**

📌 Follow the instructions from the official source:
🔗 [Gazebo Installation Guide](https://gazebosim.org/docs/fortress/install_ubuntu_src/)

---

### **2️⃣ Install Mavros**
    sudo apt-get install ros-humble-mavros ros-humble-mavros-extras
---

### **3️⃣ GeographicLib Dataset**
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    chmod +x install_geographiclib_datasets.sh
    sudo ./install_geographiclib_datasets.sh

### **4️⃣ Install PX4 Autopilot**
    git clone https://github.com/PX4/px4_msgs.git -b -recursive release/1.15
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
    Sudo reboot  
**Note:** You have to reboot your PC for this to take effect
---
## 🚀 Building Packages for PX4
    git clone https://github.com/PX4/px4_msgs.git -b release/1.15
⚠ **Note:** You must clone `px4_msgs` into the `src` directory to avoid build errors.


### **2️⃣ Install or Update Necessary Python Packages**
    pip3 install empy==3.3.4
    pip3 install setuptools==58.2.0 
### ** 3️⃣ Build the package**
    colcon build --packages-select px4_msgs
### **4️⃣ Source the Workspace**
    source install/setup.bash



---
## ✅ Notes

- Always **source** your workspace after building.
- Double-check **dependencies** if you encounter errors.
- Follow **specific package versions** to prevent build issues.
🚀 Running the Project

Once all prerequisites are installed, you can launch different components of the project using the following commands:

### **1️⃣ run the NLP Algorithm **
    ros2 run controller_pro trajectory_mavros.py
### **2️⃣ bridge PX4 with mavros **
    ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"





