import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the share directory for the package
    pkg_share = get_package_share_directory('pan_tilt_control')
    
    # URDF file path
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf')
    with open(urdf_file_path, 'r') as f:
        robot_description = f.read()

    # RViz config file path
    rviz_config_path = os.path.join(pkg_share, 'config', 'pan_tilt.rviz')

    # Declare the serial_port launch argument
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for the Arduino connection'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Pan-Tilt Driver Node
    pan_tilt_driver_node = Node(
        package='pan_tilt_control',
        executable='driver_node',
        name='pan_tilt_driver_node',
        output='screen',
        parameters=[{'serial_port': LaunchConfiguration('serial_port')}]
    )

    # Joint State Publisher GUI Node (for testing)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        serial_port_arg,
        pan_tilt_driver_node,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])