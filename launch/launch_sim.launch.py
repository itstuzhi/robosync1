# launch_sim.launch.py  (Solution B)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robosync1'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # 1) dump robot_description to a temp file, then start a repeating publisher to /robot_description
    # The publisher will keep publishing until the launch is killed
    start_publisher_cmd = [
        'bash', '-lc',
        # Sleep to let rsp set the parameter, then dump it and start publishing at 1Hz
        'sleep 1.5; ros2 param get /robot_state_publisher robot_description > /tmp/robot.urdf || true; ' \
        'python3 - <<PY\n' \
        'import time\n' \
        'from std_msgs.msg import String\n' \
        'import rclpy\n' \
        'rclpy.init()\n' \
        'node = rclpy.create_node("robot_description_publisher")\n' \
        'pub = node.create_publisher(String, "/robot_description", 10)\n' \
        's = open("/tmp/robot.urdf","r").read()\n' \
        'msg = String()\n' \
        'msg.data = s\n' \
        'try:\n' \
        '  while rclpy.ok():\n' \
        '    pub.publish(msg)\n' \
        '    time.sleep(1.0)\n' \
        'except KeyboardInterrupt:\n' \
        '  pass\n' \
        'node.destroy_node(); rclpy.shutdown()\n' \
        'PY'
    ]

    publisher_proc = ExecuteProcess(cmd=start_publisher_cmd, output='screen')

    # spawn_entity using topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bot1'],
        output='screen'
    )

    # Launch: start rsp & gazebo immediately; after a short delay, start the publisher and the spawner
    spawn_after = TimerAction(period=3.0, actions=[publisher_proc, spawn_entity])

    return LaunchDescription([rsp, gazebo, spawn_after])
