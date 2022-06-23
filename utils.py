from scipy.spatial.transform import Rotation as R
import rowan
import numpy as np
import sys

def convert_body_to_world(body_acc, world_orientation):
    """Converting linear acceleration in body frame to world frame"""
    r = R.from_quat(world_orientation)
    world_acc = r.apply(body_acc)
    return world_acc - np.array([0, 0, -9.81])

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


def get_translation_from_imu(initial_velocity, cur_accel, timestep):
    """ Calculate displacement using kinematics equation s = ut + 1/2 at^2"""
    return initial_velocity * timestep + 1/2 * cur_accel * timestep**2

def integrate_angular_velocity(prev_orientation, angular_vel, timestep):
    """Integrate angular velocity based on previous quartenion rotation"""
    return rowan.normalize(rowan.calculus.integrate(rowan.normalize(prev_orientation), angular_vel, timestep))

def isRotationMatrix(M):
    """ Checks whether a rotation matrix is orthogonal, and has a determinant of 1 (tolerance of 1e-3)"""
    tag = False
    I = np.identity(M.shape[0])
    check1 = np.isclose((np.matmul(M, M.T)), I, atol=1e-3)
    check2 = np.isclose(np.linalg.det(M),1,abs_tol=1e-3)
    if check1.all()==True and check2==True: tag = True
    return tag

def SkewSym(a):
    """ Converts a 1x3 vector to a 3x3 skew symmetric matrix"""
    a_tilda = np.array([[0, -a[2], a[1]],
                        [a[2], 0, a[0]],
                        [-a[1], a[0], 0]])
    return a_tilda

def RevSkewSym(a_tilda):
    """ Converts a 3x3 skew symmetric matrix to a 1x3 vector"""
    if a_tilda[0][0] != 0 or a_tilda[1][1] != 0 or a_tilda[2][2] != 0:
        sys.exit('Error: Matix is not skew symmetric')
    a = np.array([a_tilda[1][2], a_tilda[0][2], a_tilda[1][0]])
    return a

def EulerInt(A0,ω,r0,v,t_step,t_end):
    """ Performs Euler Integration to determine position and velocity given initial conditions, 1st derivatives, time step, and end time"""
    At=A0
    Ats=[]
    rt=r0
    rts=[]
    ω_tilda = SkewSym(ω)
    for t in range(int(t_end/t_step)):
        ω = At @ RevSkewSym(ω_tilda)
        ω_tilda = SkewSym(ω)
        At = At + t_step*(-ω_tilda @ At)
        correction_matrix = (3*np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) - (At @ At.T))/2 # correction matrix to ensure that the rotation matrix is orthogonal
        At = correction_matrix @ At
        if isRotationMatrix(At)==False:
            sys.exit('Error: Chosen integration time step is too large - try a smaller value (generally a step of <=1e-2 is recommended)')
        Ats.append(At)
        rt = rt + t_step*(At@v)
        rts.append(rt)
    return Ats,rts

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

def calc_abs_orientation(q0, q_rel):
    vos_q = rowan.multiply(q0, q_rel)
    return vos_q

def calc_rel_pose(p0, p1):
    """
    calculates VO (in the world frame) from 2 poses
    :param p0: N x 7
    :param p1: N x 7
    """
    # print(p0, p1)
    vos_t = p1[:3] - p0[:3]
    vos_q = calc_rel_orientation(p0[3:], p1[3:])
    return np.concatenate((vos_t, vos_q))

def calc_abs_pose(p0, p_rel):
    """
    calculates VO (in the world frame) from 2 poses
    :param p0: N x 7
    :param p1: N x 7
    """
    # print(p0, p1)
    vos_t = p_rel[:3] + p0[:3]
    vos_q = calc_abs_orientation(p0[3:], p_rel[3:])
    return np.concatenate((vos_t, vos_q))

if __name__ == "__main__":
    q1 = np.array([0.2209424, 0.2209424, 0.2209424, 0.9238795 ])
    q2 = np.array([0.2439988, 0.2439988, 0.2439988, 0.9063078 ])
    x1 = np.array([1, 2, 3])
    x2 = np.array([3, 3, 3])
    p1 = np.concatenate((x1, q1))
    p2 = np.concatenate((x2, q2))
    # print(qinv(q1))
    # print(rowan.inverse(q1))
    # print(normalize(q2))
    # print(rowan.normalize(q2))
    # print(rowan.multiply(q1, q2))
    # print(qmult(q1, q2))
    print(calc_rel_pose(p1, p2))
    print(calc_abs_pose(p1, calc_rel_pose(p1, p2)))