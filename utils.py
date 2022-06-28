from scipy.spatial.transform import Rotation as R
import rowan
from ahrs.filters import AngularRate
import numpy as np
import sys

def convert_body_to_world(body_val, world_orientation):
    """Converting accel/angular_velocity in body frame to world frame"""
    r = R.from_quat(world_orientation)
    world_val = r.apply(body_val)
    return world_val

def convert_body_to_world_remove_grav(body_acc, world_orientation):
    """Converting linear acceleration in body frame to world frame 
    and remove acceleration from gravity"""
    return convert_body_to_world(body_acc, world_orientation) - np.array([0, 0, -9.81])

def calc_quat_angle_error_single(label, pred):
    q1 = np.array(pred / np.linalg.norm(pred))
    q2 = np.array(label / np.linalg.norm(label))
    d = np.abs(np.sum(np.multiply(q1,q2))) # Here we have abs()

    d = np.clip(d, a_min=-1, a_max=1)
    error = 2 * np.degrees(np.arccos(d))
    return error

def obtain_relative_orientation(wxyz1, wxyz2):
    """Calculate relative orientation by converting to rotation matrix"""
    # Change to xyzw format that rowan uses and convert absolute poses to rotation matrix
    rot_mat1 = R.from_quat(np.hstack([wxyz1[1:], wxyz1[0]])).as_matrix()
    rot_mat2 = R.from_quat(np.hstack([wxyz2[1:], wxyz2[0]])).as_matrix()

    # Obtain Relative orientation 
    # relative_q = (rot_mat2.inv() * rot_mat1).as_quat()
    relative_q = R.from_matrix(np.matmul(np.transpose(rot_mat1), rot_mat2)).as_quat()
    relative_q = np.hstack([relative_q[3:], relative_q[:3]]) # Convert to wxyz format
    return relative_q

def obtain_relative_pose(absolute_pose_c1, absolute_pose_c2):
    """Obtain relative pose between two absolute poses"""
    xyz1, wxyz1 = absolute_pose_c1[:3], absolute_pose_c1[3:]
    xyz2, wxyz2 = absolute_pose_c2[:3], absolute_pose_c2[3:]

    # Obtain Relative poses
    relative_t = xyz2 - xyz1 
    relative_q = obtain_relative_orientation(wxyz1, wxyz2)

    return np.hstack([relative_t, relative_q])

def obtain_absolute_orientation(wpqr2, wpqr_rel):
    # Change to xyzw format that rowan uses and convert absolute poses to rotation matrix
    # rot_mat2 = R.from_quat(np.hstack([wpqr2[1:], wpqr2[0]]))
    # rot_mat_rel = R.from_quat(np.hstack([wpqr_rel[1:], wpqr_rel[0]]))
    rot_mat2 = R.from_quat(np.hstack([wpqr2[1:], wpqr2[0]])).as_matrix()
    rot_mat_rel = R.from_quat(np.hstack([wpqr_rel[1:], wpqr_rel[0]])).as_matrix()

    # Obtain Absolute Pose 
    # r1 = (rot_mat_rel*rot_mat2).as_quat()
    r1 = R.from_matrix(np.matmul(rot_mat2,np.linalg.inv(rot_mat_rel))).as_quat()
    r1 = np.hstack([r1[3:], r1[:3]])
    return r1

def obtain_absolute_pose(absolute_pose_c2, relative_pose):
    xyz2, wpqr2 = absolute_pose_c2[:3], absolute_pose_c2[3:]
    xyz_rel, wpqr_rel = relative_pose[:3], relative_pose[3:]

    # Obtain Absolute Pose
    t1 = xyz2 - xyz_rel 
    r1 = obtain_absolute_orientation(wpqr2, wpqr_rel)
    return np.hstack([t1, r1])

def calc_displacement(vel: float, accel: float, timestep: float) -> float:
    """ Calculate displacement using kinematics equation s = ut + 1/2 at^2"""
    return vel * timestep + 1/2 * accel * timestep ** 2 

def integrate_angular_velocity(prev_orientation, angular_vel, timestep):
    """Integrate angular velocity based on previous quartenion rotation"""
    return rowan.normalize(rowan.calculus.integrate(rowan.normalize(prev_orientation), angular_vel, timestep))

def integrate_angular_vel_body(initial_q, ang_vel_body, timesteps):
    cur_q =  initial_q
    angular_rate = AngularRate()
    for idx, (ang_vel_x, ang_vel_y, ang_vel_z) in enumerate(ang_vel_body):
        cur_q = angular_rate.update(cur_q, 
                                    np.hstack([ang_vel_x,
                                               ang_vel_y, 
                                               ang_vel_z]),
                                    'closed', 
                                    1,
                                    timesteps[idx])
        cur_q = rowan.normalize(cur_q)
    return cur_q


def normalize(x, p=2, dim=0):
    """
    Divides a tensor along a certain dim by the Lp norm
    :param x:
    :param p: Lp norm
    :param dim: Dimension to normalize along
    :return:
    """
    xn = np.linalg.norm(x)
    x = x / xn
    return x

def qmult(q1, q2):
    """
    Multiply 2 quaternions
    :param q1: Tensor N x 4
    :param q2: Tensor N x 4
    :return: quaternion product, Tensor N x 4
    """
    q1s, q1v = q1[:1], q1[1:]
    q2s, q2v = q2[:1], q2[1:]
    qs = q1s * q2s - np.dot(q1v, q2v)
    qv = q2s * q1v + q1s*q2v + np.cross(q1v, q2v)
    q = np.concatenate((qs, qv))
    # normalize
    q = normalize(q, dim=1)

    return q


def qinv(q):
    """
    Inverts quaternions
    :param q: N x 4
    :return: q*: N x 4
    """
    q_inv = np.concatenate((q[:1], -q[1:]))
    return q_inv

def calc_rel_orientation(q0, q1):
    vos_q = rowan.multiply(rowan.inverse(q0), q1)
    if vos_q[0] < 0:
        vos_q = -vos_q
    return vos_q

