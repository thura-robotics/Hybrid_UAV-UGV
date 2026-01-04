# How hybrid_robot_hardware Works - Complete Technical Walkthrough

## System Architecture Overview

```
Your Application (Python/C++)
    ↓ publishes to /position_controller/commands
ROS 2 Controllers (position_controller)
    ↓ writes to command interfaces
ros2_control_node (50 Hz control loop)
    ↓ calls read()/write()
ST3215HardwareInterface (C++)
    ↓ ROS 2 service calls
st3215_service_node.py (Python)
    ↓ Python library calls
st3215 Library
    ↓ Serial communication
ST3215 Servos (Hardware)
```

---

## Component Breakdown

### 1. **st3215 Python Library** (Low-level driver)

**Location:** `/home/eisan/Hybrid_UAV-UGV/software/st3215_driver/st3215/`

**What it does:**
- Direct serial communication with ST3215 servos
- Sends/receives packets over RS485/TTL
- Implements Feetech servo protocol

**Key methods:**
```python
ST3215(port)              # Initialize serial connection
ListServos()              # Scan for servos on bus
ReadPosition(servo_id)    # Read position in ticks (0-4095)
WritePosition(id, ticks)  # Write target position
SetMode(id, mode)         # 0=position, 1=velocity
SetSpeed(id, speed)       # Max speed
```

**Data format:**
- Position: 0-4095 ticks = 0-360° (12-bit resolution)
- Serial: 1Mbps baud rate, 8N1
- Protocol: Feetech SCServo protocol

---

### 2. **st3215_service_node.py** (Python ROS 2 Service Wrapper)

**Location:** `hybrid_robot_hardware/st3215_service_node.py`

**What it does:**
- Wraps st3215 library in ROS 2 services
- Runs as independent Python process
- Handles servo initialization and configuration

**Initialization sequence:**
```python
1. Connect to serial port (/dev/ttyUSB0)
2. Scan for servos → ListServos()
3. Configure each servo:
   - SetMode(id, 0)           # Position mode
   - SetSpeed(id, 2400)       # Fast speed
   - SetAcceleration(id, 50)  # High acceleration
4. Create ROS 2 services:
   - /st3215/read_positions
   - /st3215/write_positions
```

**Service handlers:**

**ReadPositions:**
```python
Request:  servo_ids = [1, 3, 4]
Process:  for each id: pos = servo.ReadPosition(id)
Response: positions = [1496, 2987, 2991]  # ticks
          success = True
```

**WritePositions:**
```python
Request:  servo_ids = [1, 3, 4]
          positions = [2048, 2048, 2048]  # ticks
Process:  for each id: servo.WritePosition(id, pos)
Response: success = True
```

---

### 3. **ST3215HardwareInterface** (C++ ROS 2 Control Plugin)

**Location:** `src/st3215_hardware_interface.cpp`

**What it does:**
- Implements ROS 2 Control `SystemInterface`
- Creates service clients to call Python node
- Converts between radians (ROS) and ticks (servo)

**Lifecycle:**

**on_init():**
```cpp
- Parse URDF parameters (serial_port, servo_ids)
- Resize state/command vectors
- Verify joint configuration
```

**on_configure():**
```cpp
- Create ROS 2 node for service clients
- Create clients:
  * read_client_ → /st3215/read_positions
  * write_client_ → /st3215/write_positions
- Wait for services (10s timeout)
```

**on_activate():**
```cpp
- Read initial positions from servos
- Convert ticks → radians
- Initialize command = current position
```

**read() - Called every 20ms (50 Hz):**
```cpp
1. Create ReadPositions request
   request.servo_ids = [1, 3, 4]

2. Call service (async with 100ms timeout)
   
3. Receive response
   response.positions = [1496, 2987, 2991]  # ticks

4. Convert to radians
   hw_positions_[0] = (1496 / 4096.0) * 2π = 2.29 rad
   hw_positions_[1] = (2987 / 4096.0) * 2π = 4.58 rad
   hw_positions_[2] = (2991 / 4096.0) * 2π = 4.59 rad

5. These values are now available to controllers
```

**write() - Called every 20ms (50 Hz):**
```cpp
1. Get commands from controllers
   hw_position_commands_ = [1.57, 0.0, 1.57]  # radians

2. Convert to ticks
   ticks[0] = (1.57 / 2π) * 4096 = 1024
   ticks[1] = (0.0 / 2π) * 4096 = 0
   ticks[2] = (1.57 / 2π) * 4096 = 1024

3. Create WritePositions request
   request.servo_ids = [1, 3, 4]
   request.positions = [1024, 0, 1024]

4. Call service (async with 100ms timeout)

5. Servos move to new positions!
```

**Unit conversion:**
```cpp
ticks_to_radians(int ticks):
  return (ticks / 4096.0) * 2π

radians_to_ticks(double rad):
  return (rad / 2π) * 4096
  clamp to [0, 4095]
```

---

### 4. **ROS 2 Control Loop** (ros2_control_node)

**Frequency:** 50 Hz (every 20ms)

**Control loop:**
```
Loop (50 Hz):
  1. read()  → Get current positions from hardware
  2. update() → Controllers compute new commands
  3. write() → Send commands to hardware
```

**State interfaces (read):**
```
servo_joint_1/position → 2.29 rad
servo_joint_1/velocity → 0.0
servo_joint_1/effort   → 0.0
servo_joint_3/position → 4.58 rad
...
```

**Command interfaces (write):**
```
servo_joint_1/position ← 1.57 rad (from controller)
servo_joint_3/position ← 0.0 rad
servo_joint_4/position ← 1.57 rad
```

---

### 5. **Controllers**

**joint_state_broadcaster:**
- Reads state interfaces
- Publishes to `/joint_states` topic
- Other nodes can subscribe to get current positions

**position_controller:**
- Subscribes to `/position_controller/commands`
- Receives: `Float64MultiArray{data: [1.57, 0.0, 1.57]}`
- Writes to command interfaces
- Hardware interface sends to servos

---

## Complete Data Flow Example

**User sends command:**
```bash
ros2 topic pub /position_controller/commands \
  std_msgs/msg/Float64MultiArray "data: [1.57, 0.0, 1.57]"
```

**Step-by-step:**

1. **position_controller receives message**
   - data = [1.57, 0.0, 1.57] radians

2. **Controller writes to command interfaces**
   - servo_joint_1/position ← 1.57
   - servo_joint_3/position ← 0.0
   - servo_joint_4/position ← 1.57

3. **ros2_control_node calls write() (next cycle)**
   - hw_position_commands_ = [1.57, 0.0, 1.57]

4. **C++ Hardware Interface converts to ticks**
   - [1024, 0, 1024]

5. **C++ calls Python service**
   - Service: `/st3215/write_positions`
   - Request: {servo_ids: [1,3,4], positions: [1024,0,1024]}

6. **Python service node receives request**
   - Calls `servo.WritePosition(1, 1024)`
   - Calls `servo.WritePosition(3, 0)`
   - Calls `servo.WritePosition(4, 1024)`

7. **st3215 library sends serial packets**
   - Packet format: [Header, ID, Length, Cmd, Position_L, Position_H, Checksum]
   - Sent over RS485 at 1Mbps

8. **Servos receive and execute**
   - Servo 1 moves to position 1024 (90°)
   - Servo 3 moves to position 0 (0°)
   - Servo 4 moves to position 1024 (90°)

9. **Next read() cycle (20ms later)**
   - C++ calls `/st3215/read_positions`
   - Python reads actual positions
   - Converts back to radians
   - Published to `/joint_states`

---

## Timing Analysis

**Single control cycle (50 Hz = 20ms):**
```
read():  ~2-5ms  (service call + serial read)
update(): ~0.5ms (controller computation)
write(): ~2-5ms  (service call + serial write)
Total:   ~5-11ms per cycle
```

**Service overhead:**
- Service call: ~1-2ms
- Serial I/O: ~1-5ms per servo
- Total latency: acceptable for 50 Hz control

---

## Key Files

**Python:**
- `st3215_service_node.py` - Service wrapper
- `st3215/st3215.py` - Low-level driver

**C++:**
- `st3215_hardware_interface.cpp` - ROS 2 Control plugin
- `st3215_hardware_interface.hpp` - Header

**Config:**
- `controllers.yaml` - Controller parameters
- `hybrid_robot.urdf.xacro` - Robot description
- `robot_control.launch.py` - Launch file

**Services:**
- `srv/ReadPositions.srv` - Read service definition
- `srv/WritePositions.srv` - Write service definition

---

## Summary

The system uses a **layered architecture**:

1. **Application Layer** - Your Python/C++ code
2. **Control Layer** - ROS 2 Control (controllers, hardware interface)
3. **Service Layer** - ROS 2 services (language-agnostic IPC)
4. **Driver Layer** - Python st3215 library
5. **Hardware Layer** - ST3215 servos

**Benefits:**
- ✅ Clean separation of concerns
- ✅ Easy to debug (each layer independent)
- ✅ Language flexibility (Python + C++)
- ✅ Standard ROS 2 patterns
- ✅ Full ROS 2 Control integration
