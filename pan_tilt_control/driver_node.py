import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point  # Import Point message
import serial
import threading
import time
import math

# --- DYNAMIXEL CONFIGURATION ---
DXL_CENTER_POS = 2048
DXL_POS_SCALE_RAD = 4095.0 / (2.0 * math.pi)

# --- USER DEFINED LIMITS (Raw DXL Values 0-4095) ---
PAN_MIN_DXL = 0
PAN_MAX_DXL = 2700
TILT_MIN_DXL = 682
TILT_MAX_DXL = 2048

def rad_to_dynamixel(rad):
    return int(DXL_CENTER_POS + rad * DXL_POS_SCALE_RAD)

def dynamixel_to_rad(pos):
    return (pos - DXL_CENTER_POS) / DXL_POS_SCALE_RAD

class PanTiltDriverNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_driver_node')

        # --- PARAMETERS ---
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.get_logger().info(f"Attempting connection: {serial_port} @ {baud_rate} baud")

        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
            self.serial_conn.reset_input_buffer()
            self.get_logger().info("Connected to Arduino Serial Adapter.")
        except serial.SerialException as e:
            self.get_logger().error(f"Could not open serial port {serial_port}: {e}")
            return

        # --- SUBSCRIBERS ---
        # Keep these for manual debugging/testing if needed
        self.pan_sub = self.create_subscription(Float64, '/pan_goal', self.pan_goal_callback, 10)
        self.tilt_sub = self.create_subscription(Float64, '/tilt_goal', self.tilt_goal_callback, 10)

        # NEW: Subscriber for 3D Target Point
        self.point_sub = self.create_subscription(Point, '/cmd_point', self.point_callback, 10)

        # --- PUBLISHER ---
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # --- THREADING ---
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['pan_joint', 'tilt_joint']
        self.joint_state_msg.position = [0.0, 0.0]

    def send_serial_command(self, axis, rad):
        """Helper to send command to Arduino."""
        if not hasattr(self, 'serial_conn') or not self.serial_conn.is_open:
            return

        try:
            goal_dxl = rad_to_dynamixel(rad)

            if axis == 'P': # PAN LIMITS
                if goal_dxl < PAN_MIN_DXL:
                    self.get_logger().warn(f"Pan Goal {goal_dxl} clamped to {PAN_MIN_DXL}.")
                    goal_dxl = PAN_MIN_DXL
                elif goal_dxl > PAN_MAX_DXL:
                    self.get_logger().warn(f"Pan Goal {goal_dxl} clamped to {PAN_MAX_DXL}.")
                    goal_dxl = PAN_MAX_DXL
            
            elif axis == 'T': # TILT LIMITS
                if goal_dxl < TILT_MIN_DXL:
                    self.get_logger().warn(f"Tilt Goal {goal_dxl} clamped to {TILT_MIN_DXL}.")
                    goal_dxl = TILT_MIN_DXL
                elif goal_dxl > TILT_MAX_DXL:
                    self.get_logger().warn(f"Tilt Goal {goal_dxl} clamped to {TILT_MAX_DXL}.")
                    goal_dxl = TILT_MAX_DXL

            command = f"{axis}{goal_dxl}\n"
            self.serial_conn.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f"Failed to send {axis} command: {e}")

    def point_callback(self, msg):
        """
        Receives a geometry_msgs/Point (x, y, z).
        Calculates Pan (Azimuth) and Tilt (Elevation).
        """
        x = msg.x
        y = msg.y
        z = msg.z

        # Avoid division by zero if target is exactly at (0,0,0)
        if x == 0 and y == 0 and z == 0:
            self.get_logger().warn("Received (0,0,0) target. Ignoring.")
            return

        # 1. Calculate Pan (Azimuth)
        # atan2(y, x) gives angle in radians from X-axis (-pi to +pi)
        pan_rad = math.atan2(y, x)

        # 2. Calculate Tilt (Elevation)
        # Distance on the XY plane
        r_xy = math.sqrt(x*x + y*y)
        # atan2(z, r_xy) gives angle up/down from horizon
        tilt_rad = math.atan2(z, r_xy)

        self.get_logger().info(f"Target: ({x:.2f}, {y:.2f}, {z:.2f}) -> Pan: {pan_rad:.2f} rad, Tilt: {tilt_rad:.2f} rad")

        # 3. Send Commands
        self.send_serial_command('P', pan_rad)
        self.send_serial_command('T', tilt_rad)

    # Manual callbacks now just use the helper function
    def pan_goal_callback(self, msg):
        self.send_serial_command('P', msg.data)

    def tilt_goal_callback(self, msg):
        self.send_serial_command('T', msg.data)

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                if hasattr(self, 'serial_conn') and self.serial_conn.is_open and self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8', errors='replace').strip()
                    if line.startswith('S_'):
                        parts = line.split('_')
                        if len(parts) == 3:
                            try:
                                pan_pos_dxl = int(parts[1])
                                tilt_pos_dxl = int(parts[2])
                                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                                self.joint_state_msg.position = [dynamixel_to_rad(pan_pos_dxl), dynamixel_to_rad(tilt_pos_dxl)]
                                self.joint_state_pub.publish(self.joint_state_msg)
                            except ValueError:
                                pass
            except Exception as e:
                self.get_logger().error(f"Serial loop error: {e}")
                time.sleep(1)

    def on_shutdown(self):
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()