# Agribot Precision Sprayer 
# Agribot: Precision Weed Sprayer (ROS 2 + Arduino)

An autonomous robotics project that uses Computer Vision on a Raspberry Pi 5 to detect weeds and an Arduino to actuate a precision sprayer/laser.

## Hardware Architecture
- **Brain:** Raspberry Pi 5 (Ubuntu 24.04 / ROS 2 Jazzy)
- **Actuator Controller:** Arduino Uno
- **Sensor:** USB Webcam
- **Actuators:** DC Motors, Laser Module

## Directory Structure
- `agribot_precision_sprayer/`: ROS 2 Python nodes
- `arduino_code/`: C++ Firmware for the microcontroller
- `launch/`: System startup scripts

## Installation

1. **Clone the repository:**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone [https://github.com/YOUR_USERNAME/agribot_precision_sprayer.git](https://github.com/YOUR_USERNAME/agribot_precision_sprayer.git)
