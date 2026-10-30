#!/usr/bin/env python3
"""Inspect navigation installation without opening serial or starting devices."""
from pathlib import Path
import importlib
import json
import platform
import subprocess

ROOT=Path(__file__).resolve().parent
PACKAGES=['nav2_controller','nav2_planner','nav2_navfn_planner','nav2_map_server','nav2_bt_navigator','nav2_behaviors','nav2_lifecycle_manager','nav2_velocity_smoother','dwb_core','dwb_critics','dwb_plugins','rosbag2_transport','livox_ros_driver2','fast_lio','rm27_sensor_bridge']
report={'root':str(ROOT),'system':platform.platform(),'architecture':platform.machine(),'python':platform.python_version(),'serial_opened':False,'device_started_by_this_check':False,'motion_started':False,'firmware_protocol':'Sentinel V2 A6 6A','legacy_serial_disabled':True,'dependencies':{},'pending':['Sentinel V2 0x11 navigation sender and shared serial ownership with autoaim','lidar-to-base transform and actual mounting/height verification','manual motion/scale validation and real obstacle scan projection','map and map-to-odom localization for path navigation','current V2 AUTO/STOP permission integration and physical speed/braking checks']}
for module in ['rclpy','yaml','numpy','tf2_ros','sensor_msgs.msg','nav_msgs.msg','geometry_msgs.msg','std_srvs.srv']:
    try:
        importlib.import_module(module)
        report['dependencies'][module]={'ok':True}
    except Exception as e:
        report['dependencies'][module]={'ok':False,'error':str(e)}
try:
    from ament_index_python.packages import get_package_prefix
    for name in PACKAGES:
        try: report['dependencies'][name]={'ok':True,'prefix':get_package_prefix(name)}
        except Exception as e: report['dependencies'][name]={'ok':False,'error':str(e)}
except Exception as e: report['dependencies']['ament_index_python']={'ok':False,'error':str(e)}
try:
    from rosidl_generator_py import import_type_support
    import_type_support('livox_ros_driver2')
    report['dependencies']['livox_custom_message_typesupport']={'ok':True}
except Exception as e: report['dependencies']['livox_custom_message_typesupport']={'ok':False,'error':str(e)}
libs=list((ROOT/'install/livox_driver').glob('**/liblivox_ros_driver2.so'))
if len(libs)==1:
    result=subprocess.run(['ldd',str(libs[0])],capture_output=True,text=True)
    unresolved=[line.strip() for line in result.stdout.splitlines() if 'not found' in line]
    report['dependencies']['livox_driver_linkage']={'ok':result.returncode==0 and not unresolved,'library':str(libs[0]),'unresolved':unresolved}
else: report['dependencies']['livox_driver_linkage']={'ok':False,'error':'driver shared library missing or ambiguous'}
report['installation_passed']=all(item['ok'] for item in report['dependencies'].values())
report['ready_for_vehicle_motion']=False
(ROOT/'logs/install_check.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))
raise SystemExit(0 if report['installation_passed'] else 1)
