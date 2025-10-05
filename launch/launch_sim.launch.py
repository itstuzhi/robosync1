# launch_sim.launch.py  (use this exact file; copy-paste)
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'robosync1'  # <-- keep in sync with package.xml name

    # path to top-level xacro
    xacro_path = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )

    # Include our rsp launch (so robot_state_publisher runs and has the param)
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name),
                         'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include gazebo launch (from gazebo_ros)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gazebo.launch.py')
        ]),
    )

    # Node that will spawn the robot from the 'robot_description' topic
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bot1'],
        output='screen'
    )

    # COMMAND: publish the expanded URDF to /robot_description once.
    # We use shell substitution $(xacro ...) to expand the xacro at runtime
    # and then publish it as a std_msgs/String via ros2 topic pub --once.
    # Note: the quoting below is intentionally careful for bash -lc.
    publish_urdf_cmd = (
        'bash -lc '
        + '"ros2 topic pub /robot_description std_msgs/msg/String '
        + "\\\"data: '$(xacro " + xacro_path + ")'\\\" --once" + '"'
    )

    publish_urdf = ExecuteProcess(
        cmd=[publish_urdf_cmd],
        shell=True,
        output='screen'
    )

    # Delay spawning slightly so the topic publish has time to complete
    spawn_delayed = TimerAction(
        period=1.5,  # seconds; adjust if needed
        actions=[spawn_entity]
    )

    # Order: start rsp, start gazebo, publish urdf once, then run spawn (delayed)
    return LaunchDescription([
        rsp,
        gazebo,
        publish_urdf,
        spawn_delayed
    ])
