"""Explicit vendor driver entry; no RViz, localization, TF mount, or chassis."""
import pathlib
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    path = pathlib.Path(LaunchConfiguration('user_config_path').perform(context)).resolve()
    if not path.is_file():
        raise RuntimeError('MID-360 配置文件不存在；先用真实 host/lidar IP 生成')
    xfer = int(LaunchConfiguration('xfer_format').perform(context))
    if xfer not in (0,1):
        raise RuntimeError('ROS2 此入口仅接受 xfer_format=0 PointCloud2 或 1 CustomMsg')
    frame = LaunchConfiguration('frame_id').perform(context)
    if not frame or frame.startswith('/'):
        raise RuntimeError('frame_id 须为真实雷达坐标系名称，不能带开头斜杠')
    nodes = [Node(package='livox_ros_driver2',executable='livox_ros_driver2_node',
                 name='livox_lidar_publisher',output='screen',
                 remappings=[('/livox/lidar','/livox/lidar_custom')] if xfer==1 else [],parameters=[{
                     'xfer_format':xfer,'multi_topic':0,'data_src':0,'publish_freq':10.0,
                     'output_data_type':0,'frame_id':frame,
                     'lvx_file_path':'/home/livox/livox_test.lvx',
                     'user_config_path':str(path),'cmdline_input_bd_code':'livox0000000001'}])]
    if xfer==1:
        nodes.append(Node(package='rm27_sensor_bridge',executable='sensor_bridge',name='rm27_sensor_bridge',output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('user_config_path'),
        DeclareLaunchArgument('frame_id'),
        DeclareLaunchArgument('xfer_format',default_value='1'),
        OpaqueFunction(function=start)])
