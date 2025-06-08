# Rover Control

ROS2 Humble package for controlling a differential drive rover with motor drivers and encoders.

## Hardware Requirements

- Raspberry Pi 4
- 2x L298N Motor Drivers
- 2x DC Motors with Encoders
- 12V Power Supply

## Pin Configuration

### Motor Driver 1 (Left)
- ENA: GPIO 17
- IN1: GPIO 27
- IN2: GPIO 22

### Motor Driver 2 (Right)
- ENB: GPIO 23
- IN3: GPIO 24
- IN4: GPIO 25

### Encoder 1 (Left)
- A: GPIO 5
- B: GPIO 6

### Encoder 2 (Right)
- A: GPIO 13
- B: GPIO 19

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros_ws/src
git clone <repository-url>
```

2. Install dependencies:
```bash
cd ~/ros_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select rover_control
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

1. Launch the motor driver and encoder nodes:
```bash
ros2 launch rover_control rover.launch.py
```

2. Control the motors using ROS2 topics:
```bash
# Set motor speeds (range: -100 to 100)
ros2 topic pub /motor_speeds std_msgs/msg/Int32MultiArray "data: [50, 50]"
```

3. Monitor encoder readings:
```bash
ros2 topic echo /encoder_readings
```

## License

This project is licensed under the MIT License - see the LICENSE file for details. 