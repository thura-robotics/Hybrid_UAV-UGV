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

### Clone Repository
```bash
git clone https://github.com/thura-robotics/Hybrid_UAV-UGV.git
cd Hybrid_UAV-UGV


# Hybrid UAV-UGV Robot Setup
## Prerequisites
- Ubuntu 22.04
- ROS 2 Humble
## Installation
### 1. Install ROS 2 Humble
Follow: https://docs.ros.org/en/humble/Installation.html

# Install st3215 package (goes to their ~/.local/)
cd software/morphobot_ws/src/python-st3215-main
pip3 install --user -e .

