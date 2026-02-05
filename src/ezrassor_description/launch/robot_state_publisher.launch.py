"""
Launch file for EZRASSOR robot description.

This launch file:
1. Loads the URDF from the xacro file
2. Starts robot_state_publisher to broadcast TF transforms

Usage:
  ros2 launch ezrassor_description robot_state_publisher.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    # Get the path to this package's share directory
    # After building, this will be: install/ezrassor_description/share/ezrassor_description/
    pkg_share = get_package_share_directory('ezrassor_description')

    # Path to the xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'ezrassor.urdf.xacro')

    # Process the xacro file to generate URDF XML
    # This converts our .xacro template into actual URDF
    robot_description_content = xacro.process_file(xacro_file).toxml()

    # Create the robot_state_publisher node
    # This node:
    # - Reads the URDF
    # - Publishes TF transforms for all fixed joints
    # - Publishes the robot model to /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': False,  # Set to True if using Gazebo
        }],
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
