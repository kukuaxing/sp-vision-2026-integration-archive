"""Rigid-body odometry transformations. Input pose covariance is world-frame error."""
import math
import numpy as np

def skew(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])

def quat_matrix(q):
    q=np.asarray(q,dtype=float)
    n=np.linalg.norm(q)
    if not np.isfinite(q).all() or not .8<n<1.2:raise ValueError('invalid quaternion')
    x,y,z,w=q/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

def matrix_quat(m):
    m=np.asarray(m,dtype=float)
    t=np.trace(m)
    if t>0:
        s=math.sqrt(t+1.)*2
        q=np.array([(m[2,1]-m[1,2])/s,(m[0,2]-m[2,0])/s,(m[1,0]-m[0,1])/s,.25*s])
    else:
        i=int(np.argmax(np.diag(m)));j=(i+1)%3;k=(i+2)%3
        s=math.sqrt(max(0.,1+m[i,i]-m[j,j]-m[k,k]))*2
        q=np.zeros(4);q[i]=.25*s;q[j]=(m[j,i]+m[i,j])/s;q[k]=(m[k,i]+m[i,k])/s;q[3]=(m[k,j]-m[j,k])/s
    q/=np.linalg.norm(q)
    return q

def imu_to_base(lidar_to_imu_r,lidar_to_imu_t,lidar_to_base_r,lidar_to_base_t):
    r_il=np.asarray(lidar_to_imu_r,dtype=float).reshape(3,3)
    r_bl=np.asarray(lidar_to_base_r,dtype=float).reshape(3,3)
    t_il=np.asarray(lidar_to_imu_t,dtype=float);t_bl=np.asarray(lidar_to_base_t,dtype=float)
    # I_T_B = I_T_L * inverse(B_T_L).
    r_ib=r_il@r_bl.T
    t_ib=t_il-r_ib@t_bl
    return r_ib,t_ib

def transform_odometry(position,quaternion,linear,angular,pose_cov,twist_cov,r_ib,t_ib):
    r_wi=quat_matrix(quaternion);r_bi=r_ib.T
    p=np.asarray(position)+r_wi@t_ib
    q=matrix_quat(r_wi@r_ib)
    w=r_bi@np.asarray(angular)
    v=r_bi@(np.asarray(linear)+np.cross(angular,t_ib))
    jp=np.eye(6);jp[:3,3:]=-skew(r_wi@t_ib)
    jt=np.zeros((6,6));jt[:3,:3]=r_bi;jt[:3,3:]=-r_bi@skew(t_ib);jt[3:,3:]=r_bi
    pc=jp@np.asarray(pose_cov).reshape(6,6)@jp.T
    tc=jt@np.asarray(twist_cov).reshape(6,6)@jt.T
    return p,q,v,w,pc,tc
