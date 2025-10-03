# rsp.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('robosync1')   # <- change if package.xml name differs
    xacro_file = os.path.join(pkg_dir, 'description', 'robot.urdf.xacro')  # adjust if your xacro lives elsewhere

    # Expand xacro into robot_description string
    robot_description_config = xacro.process_file(xacro_file).toxml()

    # robot_state_publisher node
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config, 'use_sim_time': False}]
    )

    # joint_state_publisher (headless) - comment / uncomment if needed
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        jsp_node
    ])
