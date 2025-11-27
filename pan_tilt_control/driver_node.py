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
# Heights relative to base (ground)
HEIGHT_BASE_TO_PAN = 0.145
HEIGHT_PAN_TO_TILT = 0.075
TILT_AXIS_Z = HEIGHT_BASE_TO_PAN + HEIGHT_PAN_TO_TILT # 0.22m

# Distance from Tilt Axis to Lidar Lens (Forward offset)
LIDAR_OFFSET = 0.043 # 43mm

# --- DYNAMIXEL CONFIGURATION ---
# Resolution: 4096 steps = 360 degrees = 2*pi radians
STEPS_PER_RAD = 4096.0 / (2.0 * math.pi)

# --- USER LIMITS (Raw DXL Values) ---
PAN_MIN_DXL = 0
PAN_MAX_DXL = 4095

TILT_MIN_DXL = 0    # Ground
TILT_MAX_DXL = 2048 # Sky

# --- MAPPING HELPER FUNCTIONS ---

def rad_to_dxl_pan(rad):
    """
    Pan is standard: 
    0 rad (Forward) = 2048
    """
    center = 2048
    return int(center + rad * STEPS_PER_RAD)

def dxl_to_rad_pan(dxl):
    center = 2048
    return (dxl - center) / STEPS_PER_RAD

def rad_to_dxl_tilt(rad):
    """
    Tilt is custom:
    0 rad (Horizon) = 1024
    +1.57 rad (Sky) = 2048
    -1.57 rad (Ground) = 0
    """
    horizon = 1024
    return int(horizon + rad * STEPS_PER_RAD)

def dxl_to_rad_tilt(dxl):
    horizon = 1024
    return (dxl - horizon) / STEPS_PER_RAD


class PanTiltDriverNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_driver_node')

        # --- PARAMETERS ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        # --- SERIAL CONNECTION ---
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info(f"Connected to {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Serial Error: {e}")
            self.serial_conn = None

        # --- SUBSCRIBERS ---
        self.point_sub = self.create_subscription(Point, '/cmd_point', self.point_callback, 10)
        # Keep manual topics for debugging if needed
        self.pan_sub = self.create_subscription(Float64, '/pan_goal', self.pan_goal_callback, 10)
        self.tilt_sub = self.create_subscription(Float64, '/tilt_goal', self.tilt_goal_callback, 10)
        
        # --- PUBLISHERS ---
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.marker_pub = self.create_publisher(Marker, '/aiming_laser', 10)

        # --- THREADING ---
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['pan_joint', 'tilt_joint']
        self.joint_state_msg.position = [0.0, 0.0]

    def point_callback(self, msg):
        """
        Calculates Pan/Tilt to aim at 3D point (x, y, z).
        """
        x = msg.x
        y = msg.y
        z = msg.z 

        # 1. PAN (Azimuth) - Rotation around Z
        pan_rad = math.atan2(y, x)

        # 2. TILT (Elevation) - Angle from Horizon
        # Adjust Z for the height of the Tilt Axis (0.22m)
        z_relative = z - TILT_AXIS_Z
        xy_distance = math.sqrt(x*x + y*y)
        
        tilt_rad = math.atan2(z_relative, xy_distance)

        # 3. Send Commands (Hardware Layer)
        self.send_serial_command('P', pan_rad)
        self.send_serial_command('T', tilt_rad)

        # 4. Update Visualization (Math Layer)
        self.publish_aiming_marker(pan_rad, tilt_rad, x, y, z)

    # --- MANUAL CALLBACKS (Optional Debugging) ---
    def pan_goal_callback(self, msg):
        self.send_serial_command('P', msg.data)
    
    def tilt_goal_callback(self, msg):
        self.send_serial_command('T', msg.data)

    def send_serial_command(self, axis, rad):
        if not self.serial_conn: return
        try:
            if axis == 'P': 
                goal_dxl = rad_to_dxl_pan(rad)
                # Clamp Pan
                goal_dxl = max(PAN_MIN_DXL, min(PAN_MAX_DXL, goal_dxl))
                self.serial_conn.write(f"P{goal_dxl}\n".encode('utf-8'))
                
            elif axis == 'T': 
                goal_dxl = rad_to_dxl_tilt(rad)
                # Clamp Tilt
                goal_dxl = max(TILT_MIN_DXL, min(TILT_MAX_DXL, goal_dxl))
                self.serial_conn.write(f"T{goal_dxl}\n".encode('utf-8'))
                
        except Exception: pass

    def publish_aiming_marker(self, pan, tilt, target_x, target_y, target_z):
        """ Draw Red Laser in Rviz from Lidar Lens to Target """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "aiming_line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.01, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0) # Red Laser

        # START POINT: Calculate Forward Kinematics for Lidar Lens
        # (Where the beam originates)
        # 1. Start at Tilt Axis Center
        start_z = TILT_AXIS_Z
        
        # 2. Add the 43mm Offset vector rotated by Pan and Tilt
        # (Spherical coordinates conversion)
        # Note: LIDAR_OFFSET is along the 'visual axis'
        offset_x = LIDAR_OFFSET * math.cos(tilt) * math.cos(pan)
        offset_y = LIDAR_OFFSET * math.cos(tilt) * math.sin(pan)
        offset_z = LIDAR_OFFSET * math.sin(tilt)

        p_start = Point()
        p_start.x = offset_x
        p_start.y = offset_y
        p_start.z = start_z + offset_z

        p_end = Point()
        p_end.x = target_x
        p_end.y = target_y
        p_end.z = target_z

        marker.points = [p_start, p_end]
        self.marker_pub.publish(marker)

    def read_serial_loop(self):
        """ 
        Reads real positions from Arduino. 
        Converts DXL -> Rad for Rviz /joint_states 
        """
        while rclpy.ok():
            if self.serial_conn and self.serial_conn.in_waiting:
                try:
                    line = self.serial_conn.readline().decode().strip()
                    if line.startswith('S_'):
                        parts = line.split('_')
                        if len(parts) == 3:
                            pan_dxl = int(parts[1])
                            tilt_dxl = int(parts[2])

                            # Convert DXL back to Rads using specific mappings
                            pan_rad = dxl_to_rad_pan(pan_dxl)
                            tilt_rad = dxl_to_rad_tilt(tilt_dxl)
                            
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