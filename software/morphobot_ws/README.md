# Hybrid Robot Workspace - Quick Start

## Setup (First Time)

Build the workspace:
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source /opt/ros/humble/setup.bash
colcon build
```

## Daily Usage

### Terminal 1: Launch MAVROS
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
./launch_mavros.sh
```

### Terminal 2: Run PX4 RC Bridge
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
./run_px4_bridge.sh
```

### Terminal 3: Run Your Control Nodes
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source setup_workspace.sh

# Mode manager
ros2 run control mode_manager_node

# UGV control
ros2 run control ugv_control_node

# Morphing control
ros2 run control morphing_control_node
```

## Monitoring

### Check PX4 RC Input (Raw PWM)
```bash
source setup_workspace.sh
ros2 topic echo /mavros/rc/in
```

### Check Normalized RC Channels
```bash
source setup_workspace.sh
ros2 topic echo /rc/channels
```

### Check Robot Mode
```bash
source setup_workspace.sh
ros2 topic echo /robot/mode
```

### Check Available Topics
```bash
source setup_workspace.sh
ros2 topic list
```

## Connection Settings

Edit `launch_mavros.sh` to change PX4 connection:
- **USB**: `fcu_url:=/dev/ttyACM0:921600`
- **Telemetry**: `fcu_url:=/dev/ttyUSB0:57600`
- **Simulation**: `fcu_url:=udp://:14540@127.0.0.1:14557`

## PX4 RC Bridge

The `px4_rc_bridge` node converts MAVROS RC input to simple ROS topics:

**Input:**
- `/mavros/rc/in` - Raw PWM values (1000-2000)

**Output:**
- `/rc/channels` - Normalized values (-1.0 to 1.0)
- `/robot/mode` - Current mode (UAV/UGV/MORPH) based on channel 5

**Mode Detection:**
- Channel 5 < 1300 → UAV mode
- Channel 5 > 1700 → MORPH mode
- 1300 ≤ Channel 5 ≤ 1700 → UGV mode

