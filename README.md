# Hybrid UAV-UGV Morphobot
### Transformable Multi-Modal Robotic System for Visual Inspection

## 📌 Project Overview
This project develops a **transformable hybrid UAV-UGV platform** capable of aerial flight, ground locomotion, and mechanical morphing for inspection in unstructured environments.

## 🧠 System Capabilities
- Multi-modal mobility (UAV ↔ UGV)
- Mechanical morphing mechanism
- Autonomous navigation
- Visual inspection and perception
- PX4 / ArduPilot integration
- ROS 2 Humble based software stack

---

## 👥 Team Structure
| Role | Responsibility |
|----|---------------|
| Hnin Ei San | ROS 2, perception, vision, locomotion |
| Thura Zaw | Schematics, PCB, power systems |
| Lamin Myat | PX4, Skydroid, low-level, firmware |
| Swan Pyae Sone | CAD, FEA, manufacturing |

---

## 🐳 Getting Started (Recommended)
### Prerequisites
- Docker
- Docker Compose
- Git

# 1. Clone repository
git clone https://github.com/thura-robotics/Hybrid_UAV-UGV.git
cd Hybrid_UAV-UGV

# 2. Install Python dependencies
pip3 install --user -r requirements.txt

# 3. Install st3215_driver (pure Python package)
cd software/st3215_driver
pip3 install --user -e .

# 4. Build ROS 2 workspace
cd ../morphobot_ws
colcon build --symlink-install
source install/setup.bash


# Install ROS 2 Control packages
Run these commands to install ROS 2 Control packages system-wide:
sudo apt update
sudo apt install -y \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-state-broadcaster \
  ros-humble-velocity-controllers \
  ros-humble-position-controllers
# Verify installation
ros2 pkg list | grep controller