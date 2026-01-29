# mode_manager Package - Complete! 🎉

## Summary

Successfully created a unified `mode_manager` package containing all control logic for the hybrid UAV-UGV robot.

## Package Contents

### Three Control Nodes

1. **mode_manager_node.py** - Top-level state machine
   - Manages robot modes: CRAWL, MORPH_*, TAKEOFF, FLY, LAND, STOP
   - Publishes `/robot_mode` topic
   - Validates state transitions

2. **ugv_control_node.py** - Ground movement control
   - Subscribes to `/cmd_vel` (Twist messages)
   - Subscribes to `/robot_mode`
   - Calls `/st3215/write_velocities` for wheel control
   - Only active in CRAWL mode

3. **morphing_control_node.py** - Morphing sequences
   - Subscribes to `/robot_mode`
   - Executes predefined morphing sequences
   - Calls `/st3215/write_positions` for morphing servos
   - Publishes `/morphing_state`

## How to Run

### Terminal 1: ST3215 Service (Hardware Driver)
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run ugv_motor_driver st3215_service_node.py --ros-args --params-file src/ugv_motor_driver/config/st3215_params.yaml
```

### Terminal 2: Mode Manager
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run mode_manager mode_manager_node
```

### Terminal 3: UGV Control
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run mode_manager ugv_control_node
```

### Terminal 4: Morphing Control
```bash
cd /home/eisan/Hybrid_UAV-UGV/software/morphobot_ws
source install/setup.bash
ros2 run mode_manager morphing_control_node
```

## Testing

### Check Current Mode
```bash
ros2 topic echo /robot_mode
```

### Check Morphing State
```bash
ros2 topic echo /morphing_state
```

### Send Velocity Command (when in CRAWL mode)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" -r 10
```

## Architecture

```
mode_manager_node
    ↓ publishes /robot_mode
    ├──→ morphing_control_node → /st3215/write_positions
    └──→ ugv_control_node ─────→ /st3215/write_velocities
                                        ↓
                                st3215_service_node
                                        ↓
                                  Physical Servos
```

## Next Steps - Customization Needed

### 1. mode_manager_node.py
- [ ] Add service or topic to receive mode change requests
- [ ] Implement proper state transition validation
- [ ] Add state machine logic (valid transitions)
- [ ] Add timeout handling

### 2. ugv_control_node.py
- [ ] Implement differential drive kinematics
- [ ] Add velocity limits and scaling
- [ ] Handle multiple wheel servos
- [ ] Add odometry publishing

### 3. morphing_control_node.py
- [ ] Load morphing sequences from YAML config file
- [ ] Add smooth interpolation between positions
- [ ] Add progress tracking (0.0 to 1.0)
- [ ] Add error handling and recovery

### 4. Configuration Files (Create These)
- [ ] `config/mode_transitions.yaml` - Valid state transitions
- [ ] `config/morphing_sequences.yaml` - Morphing position sequences
- [ ] `config/ugv_params.yaml` - UGV control parameters

### 5. Launch File (Create This)
- [ ] `launch/control_system.launch.py` - Launch all nodes together

## Current Morphing Sequences

Defined in `morphing_control_node.py`:

```python
self.sequences = {
    "MORPH_START": [2048, 2048, 2048, 2048],      # Start position
    "MORPH_UPRIGHT": [2048, 1500, 2048, 2500],    # Upright configuration
    "MORPH_QUADROTOR": [1500, 1500, 2500, 2500],  # Quadrotor configuration
}
```

**Note:** These are placeholder values. You need to determine the actual positions for your robot's morphing mechanism.

## Final Package Structure

```
morphobot_ws/src/
├── ugv_motor_driver/          # Hardware layer
│   ├── nodes/
│   │   └── st3215_service_node.py
│   ├── urdf/
│   ├── config/
│   └── srv/
│
└── mode_manager/              # Control layer
    ├── mode_manager/
    │   ├── mode_manager_node.py
    │   ├── ugv_control_node.py
    │   └── morphing_control_node.py
    ├── config/ (to be created)
    └── launch/ (to be created)
```

## Build Status

✅ Package builds successfully
✅ All three nodes registered as entry points
✅ Dependencies configured correctly

The foundation is complete and ready for you to customize! 🚀
