# ST3215 Servo Driver

Python library for controlling Waveshare ST3215 serial servos.

## Features
- Position control mode
- Velocity control mode  
- PWM control mode
- Read servo status (position, voltage, current, temperature)

# Install st3215 driver
cd software/morphobot_ws/src/st3215_driver
pip3 install --user -e .

## Usage

```python
from st3215 import ST3215

# Initialize
servo = ST3215('/dev/ttyUSB0')

# Find servos
ids = servo.ListServos()

# Move servo
servo.MoveTo(servo_id=1, position=2000, speed=3000, acc=50, wait=True)

# Read position
pos = servo.ReadPosition(servo_id=1)
```

## Dependencies
- pyserial >= 3.5

## Author
Hnin Ei San

## License
MIT



## To install for uav

<!-- mav ros -->
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras ros-humble-mavros-msgs

<!-- Install GeographicLib Datasets -->
wget -qO- https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash

<!-- MicroXRCEAgent -->
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
MicroXRCEAgent udp4 -p 8888

