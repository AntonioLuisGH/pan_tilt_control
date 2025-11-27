# Pan-Tilt Control System (AAU UAV Tracking Project)

Department of Electronic Systems — Aalborg University (AAU)

## Overview
pan_tilt_control is the ROS 2 aiming subsystem for a multi‑sensor UAV tracking architecture. It accepts a 3D target point (x, y, z) and computes Pan (azimuth) and Tilt (elevation) angles while compensating for the turret’s physical offsets.

## System architecture
- Actuators: 2× Dynamixel MX106R servos  
    - ID 1 — Pan (yaw)  
    - ID 2 — Tilt (pitch)  
- Controller: Arduino Mega 2560 + Dynamixel Shield  
- Sensor: Conical LiDAR mounted perpendicular to the tilt horn  
- Input: target point topic (geometry_msgs/msg/Point) relative to turret base

## Coordinate system & kinematics
- Solves inverse kinematics for a turret with vertical and forward offsets.
- Mechanical note: Link 3 (sensor bracket) stands vertical when the LiDAR is looking horizontal. There is a 90° mechanical offset between the arm direction and the sensor gaze.

## Physical dimensions (important constants)
- Base → Pan axis: 14.5 cm  
    - driver_node.py: `HEIGHT_BASE_TO_PAN = 0.145`
- Pan → Tilt axis: 7.5 cm  
    - driver_node.py: `HEIGHT_PAN_TO_TILT = 0.075`
- Tilt → LiDAR lens (forward offset): 4.3 cm  
    - driver_node.py & URDF: `LIDAR_OFFSET = 0.043`
- Total shoulder height: 22.0 cm (approx.)

## Installation & prerequisites
- ROS 2 (Humble / Iron / Rolling)
- Python 3, pyserial
- Arduino IDE (for firmware)

## Building
Run these from your workspace:
```bash
cd ~/ros2_ws/src
# git clone <your-repo>  # clone the package here if needed
cd ~/ros2_ws
colcon build --packages-select pan_tilt_control
source install/setup.bash
```

## Firmware & communication
- Firmware: pan_tilt_arduino_firmware/
- Baud: 57600 (must match driver_node.py)
- Protocol: Dynamixel 2.0
- Safety: firmware clamps goal positions to avoid self-collision

## Notes
- Update both `driver_node.py` (limits/offsets) and `urdf/pan_tilt.urdf` (visuals/origins) if you change geometry.
- When changing `LIDAR_OFFSET`, also update the laser joint origin in the URDF:
```xml
<joint name="laser_joint" type="fixed">
    <origin xyz="0.043 0 0" rpy="0 1.5708 0"/>
</joint>
```
- Default serial port may be `/dev/ttyUSB0`; an alternate is `/dev/ttyACM0`.
- Topic for aiming: `/cmd_point` (geometry_msgs/msg/Point). Example:
```bash
ros2 topic pub --once /cmd_point geometry_msgs/msg/Point "{x: 5.0, y: 2.0, z: 3.0}"
```
- Keep driver and URDF parameters synchronized after any mechanical changes.
# Clone repository here
cd ~/ros2_ws
colcon build --packages-select pan_tilt_control
source install/setup.bash
Configuration & CustomizationIf the physical hardware is modified, you must update the parameters in two locations: the Driver Node (for math/safety) and the URDF (for visualization).1. Adjusting Dimensions & Limits (driver_node.py)Open pan_tilt_control/driver_node.py to modify the following constants:Physical GeometryChange these if you modify the 3D printed parts or mounting heights.Python# Heights in meters relative to the base (Ground)
HEIGHT_BASE_TO_PAN = 0.145  # Height of the first static pillar
HEIGHT_PAN_TO_TILT = 0.075  # Height from Pan servo to Tilt servo axis

# The forward distance from the Tilt Motor center to the LiDAR lens
LIDAR_OFFSET = 0.043        # 43mm
Safety Limits & CalibrationChange these if you re-assemble the servos in a different orientation.Python# Raw Dynamixel Values (0-4095)
PAN_MIN_DXL = 0
PAN_MAX_DXL = 4095

# Tilt limits (Calibrated so 1024 is Horizon)
TILT_MIN_DXL = 0     # Look straight down (Ground)
TILT_MAX_DXL = 2048  # Look straight up (Sky)
2. Adjusting Visualization (pan_tilt.urdf)Open urdf/pan_tilt.urdf to match the visual model to the physical changes.Link Lengths: Update the <cylinder length="..."> and <origin z="..."> tags for link_1_post or link_2_pan.Sensor Offset: Update the laser_joint origin if LIDAR_OFFSET changes:XML<joint name="laser_joint" type="fixed">
  <origin xyz="0.043 0 0" rpy="0 1.5708 0"/> 
</joint>
Usage1. Launching the SystemThis command starts the Driver Node, the Robot State Publisher, and RViz2 with the correct visualization configuration.Bashros2 launch pan_tilt_control pan_tilt.launch.py
Optional Argument: serial_port:=/dev/ttyACM0 (Default is /dev/ttyUSB0)2. Control InterfaceThe node subscribes to a 3D point topic. The system will calculate the angles required to hit that point from its current position.Topic: /cmd_pointMessage Type: geometry_msgs/msg/PointFrame: Relative to the turret base (0,0,0).Example Command (Aim 5m forward, 2m left, 3m up):Bashros2 topic pub --once /cmd_point geometry_msgs/msg/Point "{x: 5.0, y: 2.0, z: 3.0}"
3. Visualization FeaturesRobot Model: Shows the real-time position of the links based on feedback from the servos.Laser Beam: A red line in RViz drawn from the LiDAR Lens (not the motor center) to the target point. This confirms the kinematic math is correct.Green Line: Indicates the arm is Vertical (Looking Horizontal).Red Line: Indicates the arm is Horizontal (Looking Skyward).Firmware (Arduino)Located in pan_tilt_arduino_firmware/.Baud Rate: 57600 (Must match driver_node.py)Protocol: Dynamixel 2.0Safety: The firmware clamps goal positions before moving motors to prevent self-collision, even if ROS sends a bad command.