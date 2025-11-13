import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import serial
import threading
import time
import math

# DYNAMIXEL MX-106R specific values
# Resolution: 4096 (0-4095)
# Angle Range: 360 degrees
# We will map -pi to +pi radians (-180 to +180 deg)
# Center position (0 rad) = 2048
DXL_CENTER_POS = 2048
DXL_POS_SCALE_RAD = 4095.0 / (2.0 * math.pi) # Positions per radian

def rad_to_dynamixel(rad):
    """Converts radians (-pi to +pi) to DYNAMIXEL position value (0-4095)."""
    # Clamp radians to -pi and +pi
    rad = max(-math.pi, min(math.pi, rad))
    # 0 rad -> 2048
    # pi rad -> 4095
    # -pi rad -> 0
    return int(DXL_CENTER_POS + rad * DXL_POS_SCALE_RAD)

def dynamixel_to_rad(pos):
    """Converts DYNAMIXEL position value (0-4095) to radians (-pi to +pi)."""
    return (pos - DXL_CENTER_POS) / DXL_POS_SCALE_RAD

class PanTiltDriverNode(Node):
    def __init__(self):
        super().__init__('pan_tilt_driver_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        
        self.get_logger().info(f"Connecting to Arduino on {serial_port} at {baud_rate} baud...")
        
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1.0)
            time.sleep(2) # Wait for Arduino to reset after serial connection
            self.get_logger().info("Successfully connected to Arduino.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port {serial_port}: {e}")
            rclpy.shutdown()
            return

        # Subscriptions for goal positions (in radians)
        self.pan_sub = self.create_subscription(
            Float64,
            '/pan_goal',
            self.pan_goal_callback,
            10)
        
        self.tilt_sub = self.create_subscription(
            Float64,
            '/tilt_goal',
            self.tilt_goal_callback,
            10)
            
        # Publisher for current joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Start a separate thread for reading from serial
        self.serial_thread = threading.Thread(target=self.read_serial_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['pan_joint', 'tilt_joint']
        self.joint_state_msg.position = [0.0, 0.0]
        self.joint_state_msg.velocity = []
        self.joint_state_msg.effort = []

    def pan_goal_callback(self, msg):
        """Callback for receiving a new pan goal."""
        goal_rad = msg.data
        goal_dxl = rad_to_dynamixel(goal_rad)
        command = f"P{goal_dxl}\n"
        self.serial_conn.write(command.encode('utf-8'))
        # self.get_logger().info(f"Sent Pan Goal: {goal_rad:.2f} rad ({goal_dxl})")

    def tilt_goal_callback(self, msg):
        """Callback for receiving a new tilt goal."""
        goal_rad = msg.data
        goal_dxl = rad_to_dynamixel(goal_rad)
        command = f"T{goal_dxl}\n"
        self.serial_conn.write(command.encode('utf-8'))
        # self.get_logger().info(f"Sent Tilt Goal: {goal_rad:.2f} rad ({goal_dxl})")

    def read_serial_loop(self):
        """Continuously reads from the serial port to get joint states."""
        while rclpy.ok():
            try:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    
                    # Expected format from Arduino: "S_PAN_TILT"
                    # e.g., "S_2048_2048"
                    if line.startswith('S_'):
                        parts = line.split('_')
                        if len(parts) == 3:
                            try:
                                pan_pos_dxl = int(parts[1])
                                tilt_pos_dxl = int(parts[2])
                                
                                pan_pos_rad = dynamixel_to_rad(pan_pos_dxl)
                                tilt_pos_rad = dynamixel_to_rad(tilt_pos_dxl)
                                
                                # Publish the JointState message
                                self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
                                self.joint_state_msg.position = [pan_pos_rad, tilt_pos_rad]
                                self.joint_state_pub.publish(self.joint_state_msg)
                                
                            except ValueError:
                                self.get_logger().warn(f"Received malformed data: {line}")
                    # else:
                        # Optional: log other lines from Arduino for debugging
                        # self.get_logger().info(f"Arduino: {line}")

            except rclpy.executors.ExternalShutdownException:
                break
            except Exception as e:
                self.get_logger().error(f"Error in serial read loop: {e}")
                
    def on_shutdown(self):
        """Cleans up resources on node shutdown."""
        self.get_logger().info("Shutting down node, closing serial port.")
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = PanTiltDriverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()