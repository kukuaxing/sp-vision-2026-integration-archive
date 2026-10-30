from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = Path(__file__).resolve().parent.parent
    mapping = Node(
        package='fast_lio', executable='fastlio_mapping', name='fastlio_mapping',
        output='screen', parameters=[LaunchConfiguration('config_file'), {'use_sim_time': False}],
        remappings=[('/Odometry', '/lio/odometry'), ('/cloud_registered', '/lio/cloud_registered'),
                    ('/cloud_registered_body', '/lio/cloud_body'), ('/cloud_effected', '/lio/cloud_effected'),
                    ('/Laser_map', '/lio/map'), ('/path', '/lio/path')])
    base = ExecuteProcess(cmd=['/usr/bin/python3', str(root/'localization/base_odometry.py'),
                               '--root', str(root)], output='screen')
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[EmitEvent(event=Shutdown(reason=label+' exited; localization unavailable'))]))
        for action, label in [(mapping, 'FAST-LIO'), (base, 'Base odometry')]]
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=str(root/'localization/fastlio.vehicle.yaml')),
        *handlers, mapping, base])
