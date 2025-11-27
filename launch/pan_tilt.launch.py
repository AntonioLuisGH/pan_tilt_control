import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the share directory for the package
    pkg_share = get_package_share_directory('pan_tilt_control')
    
    # URDF file path
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf')
    
    # Load the URDF file
    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # Declare the serial_port launch argument
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Arduino connection'
    )

    # 1. Robot State Publisher
    # It publishes TFs (coordinate transforms).
    # Lets other code know where the camera is pointing.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Pan-Tilt Driver Node
    # Talks to Arduino, publishes REAL /joint_states, subscribes to /pan_goal
    pan_tilt_driver_node = Node(
        package='pan_tilt_control',
        executable='driver_node',
        name='pan_tilt_driver_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 57600 # Explicitly match the firmware
        }]
    )

    return LaunchDescription([
        serial_port_arg,
        pan_tilt_driver_node,
        robot_state_publisher_node
    ])