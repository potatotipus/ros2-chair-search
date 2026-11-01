"""Small Gazebo room with GUI and the installed TurtleBot 4 robot stack."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.actions import SetEnvironmentVariable, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from room_geometry import SPAWN


def generate_launch_description():
    root = Path(__file__).resolve().parent
    world = root / 'runtime' / 'room.sdf'
    if not world.is_file():
        raise RuntimeError('Run bash demo.sh prepare first.')
    tb = Path(get_package_share_directory('turtlebot4_gz_bringup'))
    tb_desc = Path(get_package_share_directory('turtlebot4_description'))
    create_desc = Path(get_package_share_directory('irobot_create_description'))
    create = Path(get_package_share_directory('irobot_create_gz_bringup'))
    resources = ':'.join([str(tb / 'worlds'), str(create / 'worlds'),
                          str(tb_desc.parent), str(create_desc.parent),
                          os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', str(world), '--gui-config', str(root / 'room_gui.config')],
        output='screen', name='room_simulator')
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(tb / 'launch' / 'turtlebot4_spawn.launch.py')),
        launch_arguments={
            'model': 'standard', 'namespace': '', 'use_sim_time': 'true',
            'rviz': 'false', 'slam': 'false', 'localization': 'false', 'nav2': 'false',
            'x': str(SPAWN[0]), 'y': str(SPAWN[1]), 'z': '0.0', 'yaw': str(SPAWN[2]),
        }.items())
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
                 arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen')
    stop = RegisterEventHandler(OnProcessExit(target_action=sim, on_exit=[
        EmitEvent(event=Shutdown(reason='Gazebo exited; shutting down robot nodes.'))]))
    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resources), stop, sim, clock, robot])
