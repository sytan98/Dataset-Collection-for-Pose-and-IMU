from scipy.spatial.transform import Rotation as R
import rowan
import numpy as np

def convert_body_to_world(body_acc, world_orientation):
    """Converting linear acceleration in body frame to world frame"""
    r = R.from_quat(world_orientation)
    world_acc = r.apply(body_acc)
    return world_acc - np.array([0, 0, -9.81])

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


def get_translation_from_imu(initial_velocity, cur_accel, timestep):
    """ Calculate displacement using kinematics equation s = ut + 1/2 at^2"""
    return initial_velocity * timestep + 1/2 * cur_accel * timestep**2

def integrate_angular_velocity(prev_orientation, angular_vel, timestep):
    """Integrate angular velocity based on previous quartenion rotation"""
    return rowan.calculus.integrate(prev_orientation, angular_vel, timestep)