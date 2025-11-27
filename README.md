# Pan-Tilt Control System (AAU UAV Tracking Project)

**Department of Electronic Systems, Aalborg University (AAU)**

## Overview

This ROS 2 package, `pan_tilt_control`, is the aiming subsystem for a multi-sensor UAV tracking architecture. It controls a custom 2-DOF turret designed to align a Conical LiDAR with incoming drones based on 3D coordinates provided by a particle filter (fusing Acoustic DoA and LiDAR data).

The system accepts a target point $(x, y, z)$ in 3D space and automatically calculates the necessary Pan (Azimuth) and Tilt (Elevation) angles, while accounting for the physical offsets of the hardware.

## System Architecture

### Hardware

- **Actuators:** 
    - 2x Dynamixel MX106R Servomotors.
        - **ID 1 (Pan):** Rotates the tower (Yaw).
        - **ID 2 (Tilt):** Rotates the sensor bracket (Pitch).
- **Controller:** Arduino Mega 2560 + Dynamixel Shield.
- **Sensor Mounting:** The LiDAR is mounted perpendicular to the tilt servo's horn.

### Coordinate System & Kinematics

The system solves the Inverse Kinematics (IK) for a turret with specific vertical and forward offsets.

- **Physical Links:**
    - Base $\rightarrow$ Pan Axis: A vertical static post of 14.5 cm.
    - Pan Axis $\rightarrow$ Tilt Axis: A vertical offset of 7.5 cm.
    - Total "Shoulder" Height: 22.0 cm from the ground.
    - Tilt Axis $\rightarrow$ LiDAR Lens: A forward offset of 4.3 cm.

**Note on Mounting:** The physical bracket (Link 3) stands vertical when the LiDAR looks horizontal. There is a 90° mechanical offset between the arm direction and the gaze direction.

## Installation

### Prerequisites

- ROS 2 (Humble/Iron/Rolling)
- Python 3
- pyserial
- Arduino IDE (for firmware)

### Building

```bash
cd ~/ros2_ws/src
# Clone repository here
cd ~/ros2_ws
colcon build --packages-select pan_tilt_control
source install/setup.bash
```

## Configuration & Customization

If the physical hardware is modified, you must update the parameters in two locations: the Driver Node (for math/safety) and the URDF (for visualization).

### 1. Adjusting Dimensions & Limits (`driver_node.py`)

Open `pan_tilt_control/driver_node.py` to modify the following constants:

#### Physical Geometry

Change these if you modify the 3D printed parts or mounting heights.

```python
# Heights in meters relative to the base (Ground)
HEIGHT_BASE_TO_PAN = 0.145  # Height of the first static pillar
HEIGHT_PAN_TO_TILT = 0.075  # Height from Pan servo to Tilt servo axis

# The forward distance from the Tilt Motor center to the LiDAR lens
LIDAR_OFFSET = 0.043        # 43mm
```

#### Safety Limits & Calibration

Change these if you re-assemble the servos in a different orientation.

```python
# Raw Dynamixel Values (0-4095)
PAN_MIN_DXL = 0
PAN_MAX_DXL = 4095

# Tilt limits (Calibrated so 1024 is Horizon)
TILT_MIN_DXL = 0     # Look straight down (Ground)
TILT_MAX_DXL = 2048  # Look straight up (Sky)
```

### 2. Adjusting Visualization (`pan_tilt.urdf`)

Open `urdf/pan_tilt.urdf` to match the visual model to the physical changes.

- **Link Lengths:** Update the `<cylinder length="...">` and `<origin z="...">` tags for `link_1_post` or `link_2_pan`.
- **Sensor Offset:** Update the `laser_joint` origin if `LIDAR_OFFSET` changes:

```xml
<joint name="laser_joint" type="fixed">
    <origin xyz="0.043 0 0" rpy="0 1.5708 0"/> 
</joint>
```

## Usage

### 1. Launching the System

This command starts the Driver Node, the Robot State Publisher, and RViz2 with the correct visualization configuration.

```bash
ros2 launch pan_tilt_control pan_tilt.launch.py
```

**Optional Argument:** `serial_port:=/dev/ttyACM0` (Default is `/dev/ttyUSB0`)

### 2. Control Interface

The node subscribes to a 3D point topic. The system will calculate the angles required to hit that point from its current position.

- **Topic:** `/cmd_point`
- **Message Type:** `geometry_msgs/msg/Point`
- **Frame:** Relative to the turret base (0,0,0).

**Example Command (Aim 5m forward, 2m left, 3m up):**

```bash
ros2 topic pub --once /cmd_point geometry_msgs/msg/Point "{x: 5.0, y: 2.0, z: 3.0}"
```

### 3. Visualization Features

- **Robot Model:** Shows the real-time position of the links based on feedback from the servos.
- **Laser Beam:** A red line in RViz drawn from the LiDAR Lens (not the motor center) to the target point. This confirms the kinematic math is correct.
- **Green Line:** Indicates the arm is Vertical (Looking Horizontal).
- **Red Line:** Indicates the arm is Horizontal (Looking Skyward).

## Firmware (Arduino)

Located in `pan_tilt_arduino_firmware/`.

- **Baud Rate:** 57600 (Must match `driver_node.py`)
- **Protocol:** Dynamixel 2.0
- **Safety:** The firmware clamps goal positions before moving motors to prevent self-collision, even if ROS sends a bad command.