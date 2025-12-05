import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker 
import serial
import threading
import time
import math

# --- PHYSICAL GEOMETRY (Meters) ---
# Distances -> wood: 10mm, wood_to_bottom: 150mm , metal_ceiling_width: 10mm, metal_to_tilt_joint: 65mm, tilt_joint_to_lidar: 73mm
HEIGHT_BASE_TO_PAN = 0.16
HEIGHT_PAN_TO_TILT = 0.075
TILT_AXIS_Z = HEIGHT_BASE_TO_PAN + HEIGHT_PAN_TO_TILT
LIDAR_OFFSET = 0.073

# --- DYNAMIXEL CONFIGURATION ---
STEPS_PER_RAD = 4096.0 / (2.0 * math.pi)

# --- USER LIMITS ---
PAN_MIN_DXL = 0
PAN_MAX_DXL = 4095
TILT_MIN_DXL = 170 # 15 degrees
TILT_MAX_DXL = 1536 # 135 degrees

def rad_to_dxl_pan(rad):
    center = 2048
    return int(center + rad * STEPS_PER_RAD)

def dxl_to_rad_pan(dxl):
    center = 2048
    return (dxl - center) / STEPS_PER_RAD

def rad_to_dxl_tilt(rad):
    horizon = 1024
    return int(horizon + rad * STEPS_PER_RAD)

def dxl_to_rad_tilt(dxl):
    horizon = 1024
    return (dxl - horizon) / STEPS_PER_RAD

class PanTiltDriverNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_driver_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info(f"Connected to {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            self.serial_conn = None

        self.point_sub = self.create_subscription(Point, '/cmd_point', self.point_callback, 10)
        self.pan_sub = self.create_subscription(Float64, '/pan_goal', self.pan_goal_callback, 10)
        self.tilt_sub = self.create_subscription(Float64, '/tilt_goal', self.tilt_goal_callback, 10)
        
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/aiming_laser', 10)

        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['pan_joint', 'tilt_joint']
        self.joint_state_msg.position = [0.0, 0.0]

    def point_callback(self, msg):
        """
        Calculates Pan/Tilt to aim at 3D point (x, y, z) with Parallax Correction.
        """
        x = msg.x
        y = msg.y
        z = msg.z 

        # 1. PAN (Azimuth)
        pan_rad = math.atan2(y, x)

        # 2. TILT (Elevation)
        # First, calculate the "ideal" angle from the shoulder to the target
        z_relative = z - TILT_AXIS_Z
        xy_distance = math.sqrt(x*x + y*y)
        distance_3d = math.sqrt(z_relative**2 + xy_distance**2) # Hypotenuse D
        
        # Base elevation angle (Center of motor -> Target)
        base_tilt = math.atan2(z_relative, xy_distance)
        
        # 3. PARALLAX CORRECTION
        # We need to tilt slightly DOWN because the sensor is ABOVE the pivot (when arm is vertical).
        # Triangle: Hypotenuse = Distance, Opposite = Lidar Offset
        # correction = asin(Offset / Distance)
        
        # Safety: Ensure we don't divide by zero or asin(>1)
        if distance_3d > LIDAR_OFFSET:
            correction_angle = math.asin(LIDAR_OFFSET / distance_3d)
            # Subtract correction because arm is "above" the look vector
            tilt_rad = base_tilt - correction_angle
        else:
            # Target is inside the robot's head range!
            self.get_logger().warn("Target too close for parallax correction!")
            tilt_rad = base_tilt

        # 4. Send Commands
        self.send_serial_command('P', pan_rad)
        self.send_serial_command('T', tilt_rad)

        # 5. Visualize
        # IMPORTANT: The marker visualizes the RESULT. 
        # Since we corrected the servo angle, the resulting laser line 
        # (calculated from kinematics) should now land EXACTLY on the target.
        self.publish_aiming_marker(pan_rad, tilt_rad, x, y, z)

    def pan_goal_callback(self, msg):
        self.send_serial_command('P', msg.data)
    
    def tilt_goal_callback(self, msg):
        self.send_serial_command('T', msg.data)

    def send_serial_command(self, axis, rad):
        if not self.serial_conn: return
        try:
            if axis == 'P': 
                goal_dxl = rad_to_dxl_pan(rad)
                goal_dxl = max(PAN_MIN_DXL, min(PAN_MAX_DXL, goal_dxl))
                self.serial_conn.write(f"P{goal_dxl}\n".encode('utf-8'))
            elif axis == 'T': 
                goal_dxl = rad_to_dxl_tilt(rad)
                goal_dxl = max(TILT_MIN_DXL, min(TILT_MAX_DXL, goal_dxl))
                self.serial_conn.write(f"T{goal_dxl}\n".encode('utf-8'))
        except Exception: pass

    def publish_aiming_marker(self, pan_gaze, tilt_gaze, target_x, target_y, target_z):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aiming_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.01, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) 

        # --- CALCULATE START POINT (The Lidar Lens) ---
        # The Arm is always 90 degrees offset from the Gaze direction.
        # If Gaze is 0 (Horizon), Arm is +90 (Vertical Up).
        # We calculate the position of the Arm Tip based on this.
        
        # Arm Angle = Gaze Angle + 90 degrees (pi/2)
        arm_tilt = tilt_gaze + (math.pi / 2.0)
        
        # Spherical coordinates for the ARM TIP
        # Z = Center + L * sin(arm_tilt)
        # XY_Proj = L * cos(arm_tilt)
        
        p_start = Point()
        p_start.z = TILT_AXIS_Z + LIDAR_OFFSET * math.sin(arm_tilt)
        
        xy_proj = LIDAR_OFFSET * math.cos(arm_tilt)
        p_start.x = xy_proj * math.cos(pan_gaze)
        p_start.y = xy_proj * math.sin(pan_gaze)

        p_end = Point()
        p_end.x = target_x
        p_end.y = target_y
        p_end.z = target_z

        marker.points = [p_start, p_end]
        self.marker_pub.publish(marker)

    def read_serial_loop(self):
        while rclpy.ok():
            if self.serial_conn and self.serial_conn.in_waiting:
                try:
                    line = self.serial_conn.readline().decode().strip()
                    if line.startswith('S_'):
                        parts = line.split('_')
                        if len(parts) == 3:
                            pan_rad = dxl_to_rad_pan(int(parts[1]))
                            tilt_rad = dxl_to_rad_tilt(int(parts[2]))
                            
                            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                            self.joint_state_msg.position = [pan_rad, tilt_rad]
                            self.joint_state_pub.publish(self.joint_state_msg)
                except: pass
            else:
                time.sleep(0.01)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PanTiltDriverNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()