#!/usr/bin/env python3
"""Publish base-centre odometry from real FAST-LIO state. Never publishes velocity commands."""
from pathlib import Path
import argparse,json,math
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry,Path as PathMsg
from geometry_msgs.msg import TransformStamped,PoseStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster
from frame_math import imu_to_base,transform_odometry,matrix_quat

class BaseOdometry(Node):
    def __init__(self,root):
        super().__init__('navigation_base_odometry')
        c=yaml.safe_load((root/'localization/fastlio.vehicle.yaml').read_text())['/**']['ros__parameters']['mapping']
        mount=json.loads((root/'sentinel_translation/mid360_mount.json').read_text())
        r_il=np.array(c['extrinsic_R']).reshape(3,3);t_il=np.array(c['extrinsic_T'])
        r_bl=np.array(mount['rotation_lidar_to_base']);t_bl=np.array([mount['translation_m'][k] for k in ('x','y','z')])
        self.r_ib,self.t_ib=imu_to_base(r_il,t_il,r_bl,t_bl)
        self.static=StaticTransformBroadcaster(self)
        static=[]
        for frame,r,t in [('livox_frame',r_il,t_il),('base_link',self.r_ib,self.t_ib)]:
            msg=TransformStamped();msg.header.frame_id='lio_imu';msg.child_frame_id=frame;msg.header.stamp=self.get_clock().now().to_msg()
            msg.transform.translation.x,msg.transform.translation.y,msg.transform.translation.z=map(float,t)
            q=matrix_quat(r)
            msg.transform.rotation.x,msg.transform.rotation.y,msg.transform.rotation.z,msg.transform.rotation.w=map(float,q)
            static.append(msg)
        self.static.sendTransform(static)
        self.pub=self.create_publisher(Odometry,'/navigation/odom',10)
        self.path_pub=self.create_publisher(PathMsg,'/navigation/path',2)
        self.status_pub=self.create_publisher(String,'/navigation/localization_status',2)
        self.sub=self.create_subscription(Odometry,'/lio/odometry',self.received,10)
        self.last_stamp=0;self.fault=None;self.count=0;self.rejected=0
        self.path=PathMsg();self.path.header.frame_id='odom'
        self.create_timer(1.,self.status)
    def status(self):
        age=(self.get_clock().now().nanoseconds-self.last_stamp)*1e-9 if self.last_stamp else None
        msg=String();msg.data=json.dumps({'received':self.count,'rejected':self.rejected,'source_age_s':age,'fresh':age is not None and -.05<=age<=.2 and self.fault is None,'fault':self.fault,'mount_transform':'nominal user mount plus official MID360 lidar/IMU extrinsics; not physically validated','ready_for_motion':False})
        self.status_pub.publish(msg)
    def received(self,m):
        if self.fault:return
        ns=m.header.stamp.sec*1000000000+m.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-ns)*1e-9
        if m.header.frame_id!='odom' or m.child_frame_id!='lio_imu':
            self.fault='unexpected_source_frames';self.get_logger().error(self.fault);return
        # Drop duplicate/out-of-order samples; a later fresh sample remains usable.
        if ns<=self.last_stamp or age>.2 or age<-.05:self.rejected+=1;return
        p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist.linear;w=m.twist.twist.angular
        values=[p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z,*m.pose.covariance,*m.twist.covariance]
        if not all(math.isfinite(x) for x in values):self.rejected+=1;return
        try:
            pos,quat,vel,omega,pc,tc=transform_odometry([p.x,p.y,p.z],[q.x,q.y,q.z,q.w],[v.x,v.y,v.z],[w.x,w.y,w.z],m.pose.covariance,m.twist.covariance,self.r_ib,self.t_ib)
        except ValueError:self.rejected+=1;return
        out=Odometry();out.header=m.header;out.child_frame_id='base_link'
        out.pose.pose.position.x,out.pose.pose.position.y,out.pose.pose.position.z=map(float,pos)
        out.pose.pose.orientation.x,out.pose.pose.orientation.y,out.pose.pose.orientation.z,out.pose.pose.orientation.w=map(float,quat)
        out.twist.twist.linear.x,out.twist.twist.linear.y,out.twist.twist.linear.z=map(float,vel)
        out.twist.twist.angular.x,out.twist.twist.angular.y,out.twist.twist.angular.z=map(float,omega)
        out.pose.covariance=[float(x) for x in pc.flat];out.twist.covariance=[float(x) for x in tc.flat]
        self.pub.publish(out);self.last_stamp=ns;self.count+=1
        if self.count%10==0:
            pose=PoseStamped();pose.header=out.header;pose.pose=out.pose.pose
            self.path.header.stamp=out.header.stamp;self.path.poses.append(pose)
            self.path.poses=self.path.poses[-3000:];self.path_pub.publish(self.path)

def main():
    p=argparse.ArgumentParser();p.add_argument('--root',type=Path,required=True);a=p.parse_args()
    rclpy.init(args=[]);node=BaseOdometry(a.root)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
