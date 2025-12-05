import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():
    pkg_share = get_package_share_directory('pan_tilt_control')
    
    # POINT TO THE NEW XACRO FILE
    xacro_file = os.path.join(pkg_share, 'urdf', 'pan_tilt.urdf.xacro')

    # Process Xacro into URDF
    robot_description_config = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # --- ARGUMENTS ---
        DeclareLaunchArgument(
            'serial_port', 
            default_value='/dev/ttyUSB0',
            description='Serial port for Arduino'
        ),
        DeclareLaunchArgument(
            'parent_frame', 
            default_value='world',
            description='Fixed frame in Rviz'
        ),
        
        # --- NODES ---
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 
                       LaunchConfiguration('parent_frame'), 'base_link']
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            # Pass the processed command result as the description
            parameters=[{'robot_description': robot_description_config}]
        ),
        
        Node(
            package='pan_tilt_control',
            executable='driver_node',
            name='driver_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}]
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen'
        # )
    ])