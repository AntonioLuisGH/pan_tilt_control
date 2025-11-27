import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import serial
import threading
import time
import math

# --- DYNAMIXEL CONFIGURATION ---
# MX-series / Dynamixel Shield default resolution: 0-4095
# Center: 2048
DXL_CENTER_POS = 2048
# Map 360 degrees (2*pi) to 4096 steps
DXL_POS_SCALE_RAD = 4095.0 / (2.0 * math.pi) 

# --- USER DEFINED LIMITS (Raw DXL Values 0-4095) ---
# Pan: 0 to 2700
PAN_MIN_DXL = 0
PAN_MAX_DXL = 2700

# Tilt: 682 to 2048
TILT_MIN_DXL = 682
TILT_MAX_DXL = 2048

def rad_to_dynamixel(rad):
    """Converts radians (-pi to +pi) to DYNAMIXEL position value (0-4095)."""
    # Basic conversion
    return int(DXL_CENTER_POS + rad * DXL_POS_SCALE_RAD)

def dynamixel_to_rad(pos):
    """Converts DYNAMIXEL position value (0-4095) to radians (-pi to +pi)."""
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
        self.pan_sub = self.create_subscription(Float64, '/pan_goal', self.pan_goal_callback, 10)
        self.tilt_sub = self.create_subscription(Float64, '/tilt_goal', self.tilt_goal_callback, 10)
            
        # --- PUBLISHER ---
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # --- THREADING ---
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['pan_joint', 'tilt_joint']
        self.joint_state_msg.position = [0.0, 0.0]

    def pan_goal_callback(self, msg):
        if not hasattr(self, 'serial_conn') or not self.serial_conn.is_open:
            return
            
        try:
            goal_rad = msg.data
            goal_dxl = rad_to_dynamixel(goal_rad)

            # --- LIMIT CHECKING PAN ---
            if goal_dxl < PAN_MIN_DXL:
                self.get_logger().warn(f"Pan Goal {goal_dxl} too low! Clamping to {PAN_MIN_DXL}.")
                goal_dxl = PAN_MIN_DXL
            elif goal_dxl > PAN_MAX_DXL:
                self.get_logger().warn(f"Pan Goal {goal_dxl} too high! Clamping to {PAN_MAX_DXL}.")
                goal_dxl = PAN_MAX_DXL

            command = f"P{goal_dxl}\n"
            self.serial_conn.write(command.encode('utf-8'))

        except Exception as e:
            self.get_logger().warn(f"Failed to send Pan command: {e}")

    def tilt_goal_callback(self, msg):
        if not hasattr(self, 'serial_conn') or not self.serial_conn.is_open:
            return

        try:
            goal_rad = msg.data
            goal_dxl = rad_to_dynamixel(goal_rad)

            # --- LIMIT CHECKING TILT ---
            if goal_dxl < TILT_MIN_DXL:
                self.get_logger().warn(f"Tilt Goal {goal_dxl} too low! Clamping to {TILT_MIN_DXL}.")
                goal_dxl = TILT_MIN_DXL
            elif goal_dxl > TILT_MAX_DXL:
                self.get_logger().warn(f"Tilt Goal {goal_dxl} too high! Clamping to {TILT_MAX_DXL}.")
                goal_dxl = TILT_MAX_DXL

            command = f"T{goal_dxl}\n"
            self.serial_conn.write(command.encode('utf-8'))

        except Exception as e:
            self.get_logger().warn(f"Failed to send Tilt command: {e}")

    def read_serial_loop(self):
        """Continuously reads from the serial port to get joint states."""
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
                                
                                pan_pos_rad = dynamixel_to_rad(pan_pos_dxl)
                                tilt_pos_rad = dynamixel_to_rad(tilt_pos_dxl)
                                
                                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                                self.joint_state_msg.position = [pan_pos_rad, tilt_pos_rad]
                                self.joint_state_pub.publish(self.joint_state_msg)
                                
                            except ValueError:
                                pass 
            except Exception as e:
                self.get_logger().error(f"Serial loop error: {e}")
                time.sleep(1) 
                
    def on_shutdown(self):
        self.get_logger().info("Shutting down, closing serial port.")
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