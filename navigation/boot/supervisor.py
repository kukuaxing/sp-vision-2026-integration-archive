"""Own the boot navigation processes, monitor live readiness, and never enable chassis motion."""
from pathlib import Path
import json,math,os,signal,socket,subprocess,time,uuid
import rclpy
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from course_registration import RegistrationStore
from startup_policy import StartupBudget

N=Path('/home/rcsdart/rm27/navigation');LOG=N/'logs/boot';LOG.mkdir(parents=True,exist_ok=True)
course_registration=RegistrationStore(N)
start=time.monotonic();started_unix=time.time();boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()
identity=uuid.uuid4().hex;owned=[];backend=None;backend_log=None;stopping=False
node=None;status_pub=None;initial_ready=None;stable_since=None;inputs={};state={};pose=None;state_rx=None;state_stamp=None
state_epoch=None;last_state_stamp=0;generation_changes=0;last_error=None
def notify(value):
    name=os.environ.get('NOTIFY_SOCKET')
    if not name:return
    if name.startswith('@'):name='\0'+name[1:]
    with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as s:s.sendto(value.encode(),name)
def atomic(path,value):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(value,indent=2));tmp.replace(path)
def status(name):
    return json.loads(subprocess.check_output(['/usr/bin/python3',str(N/f'{name}_control.py'),'status'],text=True))
def start_managed(name):
    owned.append(name)
    subprocess.run(['/usr/bin/python3',str(N/f'{name}_control.py'),'start'],check=True)
def stamp(m):return m.header.stamp.sec*10**9+m.header.stamp.nanosec
def input_cb(topic,m):
    s=stamp(m)
    if m.header.frame_id!='odom':return
    if topic=='base' and m.child_frame_id!='base_link':return
    if topic=='odom' and m.child_frame_id!='lio_imu':return
    prev=inputs.get(topic)
    if prev and s<=prev['stamp_ns']:return
    inputs[topic]={'stamp_ns':s,'received_monotonic':time.monotonic()}
def state_cb(m):
    global state,state_rx,state_stamp,state_epoch,last_state_stamp,generation_changes,stable_since
    fields=dict(x.split('=',1) for x in m.data.split(';') if '=' in x)
    epoch=(fields.get('epoch'),fields.get('generation'))
    if state_epoch is not None and epoch!=state_epoch:
        generation_changes+=1;stable_since=None;state_stamp=None;last_state_stamp=0
    state_epoch=epoch;state=fields;state_rx=time.monotonic()
    s=int(fields.get('stamp_ns','0'))
    if s>last_state_stamp:state_stamp=s;last_state_stamp=s
def pose_cb(m):
    global pose
    if m.header.frame_id!='motion_plane_candidate':return
    p=m.pose.pose.position;q=m.pose.pose.orientation
    if not all(math.isfinite(v) for v in [p.x,p.y,p.z,q.x,q.y,q.z,q.w,*m.pose.covariance]):return
    norm=sum(v*v for v in [q.x,q.y,q.z,q.w])
    if not .99<norm<1.01:return
    pose={'stamp_ns':stamp(m),'received_monotonic':time.monotonic(),'xyz':[p.x,p.y,p.z],
        'yaw_deg':math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))}
def fresh(s):return s is not None and -.1<(time.time_ns()-s)/1e9<.75
def snapshot(phase,ready=False):
    now=time.monotonic()
    course=course_registration.snapshot(pose)
    report=dict(phase=phase,localization_ready=ready,motion_enabled=False,requires_course_registration=not course['course_reference_valid'],
        instance=identity,pid=os.getpid(),boot_id=boot_id,updated_unix=time.time(),started_unix=started_unix,
        elapsed_s=now-start,first_ready_s=initial_ready,boot_uptime_s=float(Path('/proc/uptime').read_text().split()[0]),
        input_fresh={k:fresh(inputs.get(k,{}).get('stamp_ns')) for k in ['odom','cloud','base']},
        map_state=state.get('status'),pose_trusted=state.get('pose_trusted'),epoch=state_epoch,
        generation_changes=generation_changes,pose=pose,last_error=last_error,
        primary_tf_published=False,tf_topic='/navigation/map_candidate_tf',motion_commands_sent=False)
    report.update(course)
    report['course_pose_usable']=ready and course['course_reference_valid']
    atomic(LOG/'status.json',report)
    if node is not None and status_pub is not None:
        msg=String();msg.data=json.dumps(report);status_pub.publish(msg)
    return report
def stop_signal(signum,frame):
    global stopping
    stopping=True
signal.signal(signal.SIGTERM,stop_signal);signal.signal(signal.SIGINT,stop_signal)
try:
    snapshot('WAITING_ETHERNET');notify('STATUS=Waiting for radar Ethernet address')
    startup_budget=StartupBudget(start)
    last_wait_report=0.
    while not stopping:
        try:
            addresses=subprocess.check_output(['ip','-4','addr','show','dev','enp3s0'],text=True,stderr=subprocess.DEVNULL)
        except subprocess.CalledProcessError:
            addresses='' # NIC enumeration/link setup may still be in progress.
        # Static addresses alone do not establish a physically connected radar.
        try:carrier=Path('/sys/class/net/enp3s0/carrier').read_text().strip()=='1'
        except OSError:carrier=False
        if carrier and 'inet 192.168.1.2/24' in addresses:break
        now=time.monotonic()
        # Device-late boot and a detached computer wait without expiring or
        # repeatedly tearing down the service/its dependants.
        if now-last_wait_report>=.5:
            if startup_budget.network_wait_long(now):notify('STATUS=Waiting for radar cable/address; no startup expiry')
            snapshot('WAITING_ETHERNET');last_wait_report=now
        time.sleep(.2)
    if stopping:raise InterruptedError('Service stop requested')
    startup_budget.network_ready(time.monotonic())
    snapshot('STARTING_SENSORS')
    if not status('lidar')['driver_process_running']:start_managed('lidar')
    loc=status('localization')
    if not loc['localization_processes_running']:
        if loc['launch_process_running']:
            subprocess.run(['/usr/bin/python3',str(N/'localization_control.py'),'stop'],check=True,timeout=15)
        start_managed('localization')
    backend_log=(LOG/'map_backend.log').open('a')
    backend_log.write('\n--- instance '+identity+' ---\n');backend_log.flush()
    backend=subprocess.Popen([str(N/'install/z_localization/backend/lib/backend/backend_node'),
        '--ros-args','--params-file',str(N/'boot/map_backend.yaml'),'-r','__node:=map_localization',
        '-r','/tf:=/navigation/map_candidate_tf','-r','/tf_static:=/navigation/map_candidate_tf_static'],
        stdout=backend_log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('navigation_boot_monitor')
    status_pub=node.create_publisher(String,'/navigation/startup_status',10)
    subs=[node.create_subscription(typ,topic,lambda m,key=key:input_cb(key,m),qos_profile_sensor_data)
        for key,topic,typ in [('odom','/lio/odometry',Odometry),('cloud','/lio/cloud_registered',PointCloud2),('base','/navigation/odom',Odometry)]]
    subs.extend([node.create_subscription(String,'/map_localization/localization_state',state_cb,10),
        node.create_subscription(PoseWithCovarianceStamped,'/map_localization/localization_pose',pose_cb,20)])
    last_write=0;last_phase=None;input_loss_since=None
    while not stopping:
        rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
        if backend.poll() is not None:raise RuntimeError(f'Map backend exited: {backend.returncode}')
        sensors_ok=all(fresh(inputs.get(k,{}).get('stamp_ns')) for k in ['odom','cloud','base'])
        if sensors_ok:input_loss_since=None
        elif input_loss_since is None:input_loss_since=now
        if initial_ready is not None and input_loss_since is not None and now-input_loss_since>10:
            raise RuntimeError('Navigation inputs stopped for >10s after readiness; rebuilding passive sensor chain')
        map_ok=(state.get('status')=='NORMAL' and state.get('pose_trusted')=='true' and
            state.get('tf_frozen')=='false' and fresh(state_stamp) and state_rx is not None and now-state_rx<.75 and
            pose is not None and fresh(pose['stamp_ns']) and now-pose['received_monotonic']<.75)
        if sensors_ok and map_ok:
            if stable_since is None:stable_since=now
        else:stable_since=None
        ready=stable_since is not None and now-stable_since>=1.0
        phase='LOCALIZATION_READY' if ready else ('WAITING_MAP' if sensors_ok else 'WAITING_INPUTS')
        if ready and initial_ready is None:
            initial_ready=now-start
            atomic(LOG/'last_startup_receipt.json',snapshot(phase,True))
            notify('READY=1\nSTATUS=Live map localization ready; chassis motion disabled')
            print(f'Live localization ready after {initial_ready:.3f}s; motion remains disabled',flush=True)
        if now-last_write>=.25 or phase!=last_phase:
            snapshot(phase,ready);last_write=now
            if phase!=last_phase:print(f'{phase}, elapsed={now-start:.3f}s',flush=True)
            last_phase=phase
        if initial_ready is None and startup_budget.localization_expired(now):raise TimeoutError('Live localization did not become ready within 75s after radar network readiness')
except InterruptedError:
    pass
except Exception as e:
    last_error=str(e);snapshot('FAILED');notify('STATUS=Navigation startup failed: '+last_error);raise
finally:
    if backend is not None and backend.poll() is None:
        try:os.killpg(backend.pid,signal.SIGINT);backend.wait(timeout=6)
        except subprocess.TimeoutExpired:os.killpg(backend.pid,signal.SIGTERM);backend.wait(timeout=3)
        except ProcessLookupError:pass
    if backend_log:backend_log.close()
    for name in reversed(owned):
        subprocess.run(['/usr/bin/python3',str(N/f'{name}_control.py'),'stop'],timeout=10)
    if node is not None:
        snapshot('STOPPED' if last_error is None else 'FAILED');node.destroy_node();rclpy.shutdown()
