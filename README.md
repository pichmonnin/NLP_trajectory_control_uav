# UAV Obstacle Avoidance 🚀

## 📌 Prerequisites

### **Required Software**

- **ROS 2 Humble (Ubuntu 22.04)**\
  ⚠ **Note:** Setup Tool must be **version 65.5.1** to prevent build errors with `px4_msgs`.
- **Micro XRCE-DDS Agent**
- **PX4 Autopilot**
- **Gazebo Igition**
- **Joystick Driver**

---
    


## 📦 Dependencies for ROS 2 Humble
    pip3 install --user -U empy pyros-genmsg setuptools
    pip3 install kconfiglib
    pip install --user jsonschema
    pip install --user jinja2

---

## 🏗 Installation Steps

### **1️⃣ Install Gazebo**

📌 Follow the instructions from the official source:\
🔗 [Gazebo Installation Guide](https://gazebosim.org/docs/fortress/install_ubuntu_src/)

---

### **2️⃣ Install Micro XRCE-DDS Agent**
    git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    mkdir build
    cd build
    cmake ..
    make
    sudo make install 
    sudo ldconfig /usr/local/lib/
---

### **3️⃣ Install PX4 Autopilot**
    git clone https://github.com/PX4/px4_msgs.git -b release/1.15
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
## 🎮 **Joystick Driver Installation **
    git clone https://github.com/ros-drivers/joystick_drivers.git



---
## ✅ Notes

- Always **source** your workspace after building.
- Double-check **dependencies** if you encounter errors.
- Follow **specific package versions** to prevent build issues.
🚀 Running the Project

Once all prerequisites are installed, you can launch different components of the project using the following commands:

### **1️⃣ Launch the Controller **
    ros2 launch controller controller_launch.py 


### **2️⃣ Launch the Simulation **
    ros2 launch controller simulation_gaz.py 

### **3️⃣ Bridge Data and View in RViz2**
    ros2 launch controller data_bridge.py




