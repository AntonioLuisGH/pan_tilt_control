import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('pan_tilt_control')
    urdf_file = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        
        # 1. State Publisher (Handles TF for URDF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 2. Driver Node (Talks to Arduino + Publishes Laser Marker)
        Node(
            package='pan_tilt_control',
            executable='driver_node',
            name='driver_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}]
        ),

        # 3. RViz2 (Visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'pan_tilt.rviz')],
            output='screen'
        )
    ])