# launch_sim.launch.py  (Solution A)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'robosync1'

    # include rsp (which sets robot_description param)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # include gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    # command to write robot_description param to a temporary urdf file
    dump_urdf_cmd = [
        'bash', '-lc',
        # wait 1.5s so rsp has time to set the parameter (adjust if needed)
        'sleep 1.5; ' \
        'ros2 param get /robot_state_publisher robot_description > /tmp/robot.urdf || ' \
        'echo "FAIL_TO_DUMP_ROBOT_DESCRIPTION" > /tmp/robot.urdf'
    ]
    dump_urdf = ExecuteProcess(cmd=dump_urdf_cmd, output='screen')

    # spawn_entity using the file we just dumped. We will delay this action using TimerAction.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', '/tmp/robot.urdf', '-entity', 'bot1'],
        output='screen'
    )

    # run dump_urdf and spawn_entity after a short delay (3s) to ensure gazebo is up.
    spawn_after = TimerAction(period=3.0, actions=[dump_urdf, spawn_entity])

    return LaunchDescription([rsp, gazebo, spawn_after])
