"""Load the matching prior map and the known spawn pose, then start Nav2."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    runtime = Path(__file__).resolve().parent / 'runtime'
    if not (runtime / 'nav2.yaml').is_file():
        raise RuntimeError('Run bash demo.sh prepare first.')
    launch = Path(get_package_share_directory('turtlebot4_navigation')) / 'launch'
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch / 'localization.launch.py')),
        launch_arguments={'map': str(runtime / 'room.yaml'),
                          'params': str(runtime / 'localization.yaml'),
                          'use_sim_time': 'true'}.items())
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch / 'nav2.launch.py')),
        launch_arguments={'params_file': str(runtime / 'nav2.yaml'),
                          'use_sim_time': 'true'}.items())
    return LaunchDescription([localization, nav])
