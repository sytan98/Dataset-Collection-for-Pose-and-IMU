import pandas as pd
import rowan
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy import signal
from scipy import integrate
from ahrs.filters import AngularRate
import os
import argparse

def convert_body_to_world(row):
    body_acc = np.hstack([row["S_LIN_ACC_X"], row["S_LIN_ACC_Y"], row["S_LIN_ACC_Z"]]).astype(float)
    r = R.from_quat(np.hstack([row["pure_imu_Q_X"], row["pure_imu_Q_Y"], row["pure_imu_Q_Z"], row["pure_imu_Q_W"]]))
    world_acc = r.apply(body_acc)
    return world_acc - np.array([0, 0, -9.81])

def integrate_angular_vel(initial_q, ang_vel, timesteps):
    cur_q =  initial_q
    angular_rate = AngularRate()
    for idx, (ang_vel_x, ang_vel_y, ang_vel_z) in enumerate(ang_vel):
        cur_q = angular_rate.update(cur_q, 
                                    np.hstack([ang_vel_x,
                                               ang_vel_y, 
                                               ang_vel_z]),
                                    'closed', 
                                    1,
                                    timesteps[idx])
        cur_q = rowan.normalize(cur_q)
    return cur_q

def calc_displacement(vel: float, accel: float, timestep: float) -> float:
    return vel * timestep + 1/2 * accel * timestep ** 2 

def gen_imu_derived_rel_poses(dir:str , train: bool, noise_level: int, skip: int, imu_freq: int, img_freq: int) -> None:
    train_data = pd.read_csv(os.path.join(dir, 'airsim_rec.txt'), sep="\t")

    np.random.seed(123)
    if noise_level == 0 or not train:
        noise_x = 0
        noise_y = 0
        noise_z = 0
        noise_qw = 0
        noise_qx = 0
        noise_qy = 0
        noise_qz = 0
    elif noise_level == 1:
        noise_x = np.random.normal(0,0.5,len(train_data))
        noise_y = np.random.normal(0,0.5,len(train_data))
        noise_z = np.random.normal(0,0.1,len(train_data))
        noise_qw = np.random.normal(0,0.03,len(train_data))
        noise_qx = np.random.normal(0,0.03,len(train_data))
        noise_qy = np.random.normal(0,0.03,len(train_data))
        noise_qz = np.random.normal(0,0.03,len(train_data))
    elif noise_level == 2:     
        noise_x = np.random.normal(0,0.8,len(train_data))
        noise_y = np.random.normal(0,0.8,len(train_data))
        noise_z = np.random.normal(0,0.2,len(train_data))
        noise_qw = np.random.normal(0,0.05,len(train_data))
        noise_qx = np.random.normal(0,0.05,len(train_data))
        noise_qy = np.random.normal(0,0.05,len(train_data))
        noise_qz = np.random.normal(0,0.05,len(train_data))
    
    train_data["noisy_POS_X"] = noise_x + train_data["POS_X"]
    train_data["noisy_POS_Y"] = noise_y + train_data["POS_Y"]
    train_data["noisy_POS_Z"] = noise_z + train_data["POS_Z"]
    train_data["noisy_Q_W"] = noise_qw + train_data["Q_W"]
    train_data["noisy_Q_X"] = noise_qx + train_data["Q_X"]
    train_data["noisy_Q_Y"] = noise_qy + train_data["Q_Y"]
    train_data["noisy_Q_Z"] = noise_qz + train_data["Q_Z"]
    # Ensure that noisy quaternions is a valid rotation
    normalized_q = pd.DataFrame(rowan.normalize(train_data[["noisy_Q_W", "noisy_Q_X", "noisy_Q_Y", "noisy_Q_Z"]]), columns=["noisy_Q_W", "noisy_Q_X", "noisy_Q_Y", "noisy_Q_Z"])
    train_data["noisy_Q_W"] = normalized_q["noisy_Q_W"]
    train_data["noisy_Q_X"] = normalized_q["noisy_Q_X"]
    train_data["noisy_Q_Y"] = normalized_q["noisy_Q_Y"]
    train_data["noisy_Q_Z"] = normalized_q["noisy_Q_Z"]

    train_data["noisy_prev_POS_X"] = train_data["noisy_POS_X"].shift(1)
    train_data["noisy_prev_POS_Y"] = train_data["noisy_POS_Y"].shift(1)
    train_data["noisy_prev_POS_Z"] = train_data["noisy_POS_Z"].shift(1)
    train_data["noisy_prev_Q_W"] = train_data["noisy_Q_W"].shift(1)
    train_data["noisy_prev_Q_X"] = train_data["noisy_Q_X"].shift(1)
    train_data["noisy_prev_Q_Y"] = train_data["noisy_Q_Y"].shift(1)
    train_data["noisy_prev_Q_Z"] = train_data["noisy_Q_Z"].shift(1)

    # Calculate timestamp, time from start in seconds, and timestep between imu recordings
    train_data["timestamp"] = pd.to_datetime(train_data["timestamp"], unit='ns')
    train_data["seconds"] = (train_data["timestamp"] - train_data["timestamp"][0]).dt.total_seconds()
    train_data["prev_timestamp"] = train_data["timestamp"].shift(1)
    train_data["timestep"] = ((train_data["timestamp"] - train_data["prev_timestamp"]).dt.total_seconds())

    # Get previous angular velocity
    train_data["prev_S_ANG_VEL_X"] = train_data["S_ANG_VEL_X"].shift(1)
    train_data["prev_S_ANG_VEL_Y"] = train_data["S_ANG_VEL_Y"].shift(1)
    train_data["prev_S_ANG_VEL_Z"] = train_data["S_ANG_VEL_Z"].shift(1)
    
    print("---------------Getting orientation from IMU--------------")
    qw = train_data.iloc[0, :].noisy_Q_W
    qx = train_data.iloc[0, :].noisy_Q_X
    qy = train_data.iloc[0, :].noisy_Q_Y
    qz = train_data.iloc[0, :].noisy_Q_Z
    start_q = np.array([qw, qx, qy, qz])
    abs_results = [start_q]
    for row in train_data.iloc[1:,:].itertuples(index=False):
        ang_vel = [(row.prev_S_ANG_VEL_X, row.prev_S_ANG_VEL_Y, row.prev_S_ANG_VEL_Z)]
        timestep = [row.timestep]
        start_q = integrate_angular_vel(start_q, ang_vel, timestep)
        abs_results.append(start_q)
    imu_q_ahrs = pd.DataFrame(abs_results, columns=["pure_imu_Q_W", "pure_imu_Q_X", "pure_imu_Q_Y", "pure_imu_Q_Z"])

    train_data = pd.concat([train_data, imu_q_ahrs], axis=1)

    # Convert imu linear acceleration from body frame to world frame
    train_data["S_LIN_ACC_X_world"] = train_data.apply(lambda row : convert_body_to_world(row)[0], axis = 1)
    train_data["S_LIN_ACC_Y_world"] = train_data.apply(lambda row : convert_body_to_world(row)[1], axis = 1)
    train_data["S_LIN_ACC_Z_world"] = train_data.apply(lambda row : convert_body_to_world(row)[2], axis = 1)

    vel_x = integrate.cumtrapz(train_data["S_LIN_ACC_X_world"], train_data["seconds"], initial=0)
    vel_y = integrate.cumtrapz(train_data["S_LIN_ACC_Y_world"], train_data["seconds"], initial=0)
    vel_z = integrate.cumtrapz(train_data["S_LIN_ACC_Z_world"], train_data["seconds"], initial=0) 
    vel_x = signal.detrend(vel_x)
    vel_y = signal.detrend(vel_y)
    vel_z = signal.detrend(vel_z)

    # Get imu linear velocity integrated using imu acceleration corrected using noisy orientation
    train_data["S_LIN_VEL_X"] = vel_x
    train_data["S_LIN_VEL_Y"] = vel_y
    train_data["S_LIN_VEL_Z"] = vel_z
    train_data["prev_S_LIN_VEL_X"] = train_data["S_LIN_VEL_X"].shift(1)
    train_data["prev_S_LIN_VEL_Y"] = train_data["S_LIN_VEL_Y"].shift(1)
    train_data["prev_S_LIN_VEL_Z"] = train_data["S_LIN_VEL_Z"].shift(1)

    train_data["S_LIN_ACC_X"] = train_data["S_LIN_ACC_X_world"]
    train_data["S_LIN_ACC_Y"] = train_data["S_LIN_ACC_Y_world"]
    train_data["S_LIN_ACC_Z"] = train_data["S_LIN_ACC_Z_world"]
    train_data["prev_S_LIN_ACC_X"] = train_data["S_LIN_ACC_X"].shift(1)
    train_data["prev_S_LIN_ACC_Y"] = train_data["S_LIN_ACC_Y"].shift(1)
    train_data["prev_S_LIN_ACC_Z"] = train_data["S_LIN_ACC_Z"].shift(1)

    # number of imu readings for each image frame (e.g. if 100Hz IMU and 10Hz Img freq,
    # there are 10 imu readings for each image frame)
    image_frame_step = int(imu_freq / img_freq)
    # Gap between frames used in sample (rmb to change the value in 
    # MapNet.ini as well, skip = k/image_frame_step = 10)
    k  = skip * image_frame_step
    
    print("----------Getting relative translation from IMU----------")
    imu_rel_trans = []
    for i in range(k, len(train_data), image_frame_step):
        x = y = z = 0
        for j in range(i - k + 1, i + 1):
            row = train_data.iloc[j, :]
            x += calc_displacement(row.prev_S_LIN_VEL_X, row.prev_S_LIN_ACC_X, row.timestep)
            y += calc_displacement(row.prev_S_LIN_VEL_Y, row.prev_S_LIN_ACC_Y, row.timestep)
            z += calc_displacement(row.prev_S_LIN_VEL_Z, row.prev_S_LIN_ACC_Z, row.timestep)
        imu_rel_trans.append([x, y, z])

    imu_rel_trans = pd.DataFrame(imu_rel_trans, columns=["imu_rel_POS_X", "imu_rel_POS_Y","imu_rel_POS_Z"])
    imu_rel_trans["idx"] = [i for i in range(k, len(train_data), image_frame_step)]
    imu_rel_trans.set_index("idx", inplace=True)

    train_data["imu_rel_POS_X"] = imu_rel_trans["imu_rel_POS_X"]
    train_data["imu_rel_POS_Y"] = imu_rel_trans["imu_rel_POS_Y"]
    train_data["imu_rel_POS_Z"] = imu_rel_trans["imu_rel_POS_Z"]
    train_data["imu_POS_X"] = train_data["noisy_POS_X"].shift(k) + train_data["imu_rel_POS_X"]
    train_data["imu_POS_Y"] = train_data["noisy_POS_Y"].shift(k) + train_data["imu_rel_POS_Y"]
    train_data["imu_POS_Z"] = train_data["noisy_POS_Z"].shift(k) + train_data["imu_rel_POS_Z"]

    print("----------Getting relative orientation from IMU----------")
    abs_results = []
    for i in range(k, len(train_data), image_frame_step):
        qw = train_data.iloc[i - k, :].noisy_Q_W
        qx = train_data.iloc[i - k, :].noisy_Q_X
        qy = train_data.iloc[i - k, :].noisy_Q_Y
        qz = train_data.iloc[i - k, :].noisy_Q_Z
        start_q = np.array([qw, qx, qy, qz])
        ang_vels = train_data[["prev_S_ANG_VEL_X", "prev_S_ANG_VEL_Y", "prev_S_ANG_VEL_Z"]].iloc[i-k+1:i+1,:].values
        timesteps = train_data[["timestep"]].iloc[i-k+1:i+1,:].values
        cur_q = integrate_angular_vel(start_q, ang_vels, timesteps)
        abs_results.append(cur_q)

    imu_q_ahrs = pd.DataFrame(abs_results, columns=["imu_Q_W", "imu_Q_X", "imu_Q_Y", "imu_Q_Z"])
    imu_q_ahrs["idx"] = [i for i in range(k, len(train_data), image_frame_step)]
    imu_q_ahrs.set_index("idx", inplace=True)
    train_data["imu_Q_W"] = imu_q_ahrs["imu_Q_W"]
    train_data["imu_Q_X"] = imu_q_ahrs["imu_Q_Y"]
    train_data["imu_Q_Y"] = imu_q_ahrs["imu_Q_X"]
    train_data["imu_Q_Z"] = imu_q_ahrs["imu_Q_Z"]

    final_write_file = train_data.iloc[::image_frame_step,:]
    final_write_file = final_write_file[["ImageFile", "timestep", 
                                "noisy_POS_X", "noisy_POS_Y", "noisy_POS_Z", "noisy_Q_W", "noisy_Q_X", "noisy_Q_Y", "noisy_Q_Z",
                                "imu_POS_X", "imu_POS_Y", "imu_POS_Z", "imu_Q_W", "imu_Q_X","imu_Q_Y","imu_Q_Z"
                                ]]
    final_write_file.reset_index(drop=True, inplace =True)
    final_write_file.rename(columns = {'noisy_POS_X':'POS_X', 
                                       'noisy_POS_Y':'POS_Y',
                                       'noisy_POS_Z':'POS_Z',
                                       'noisy_Q_W':'Q_W',
                                       'noisy_Q_X':'Q_X',
                                       'noisy_Q_Y':'Q_Y',
                                       'noisy_Q_Z':'Q_Z' }, 
                            inplace = True)

    final_write_file.loc[0:skip-1,'imu_POS_Y'] = final_write_file.loc[0:skip-1,'POS_Y']
    final_write_file.loc[0:skip-1,'imu_POS_Z'] = final_write_file.loc[0:skip-1,'POS_Z']
    final_write_file.loc[0:skip-1,'imu_POS_X'] = final_write_file.loc[0:skip-1,'POS_X']
    final_write_file.loc[0:skip-1,'imu_Q_W'] = final_write_file.loc[0:skip-1,'Q_W']
    final_write_file.loc[0:skip-1,'imu_Q_X'] = final_write_file.loc[0:skip-1,'Q_X']
    final_write_file.loc[0:skip-1,'imu_Q_Y'] = final_write_file.loc[0:skip-1,'Q_Y']
    final_write_file.loc[0:skip-1,'imu_Q_Z'] = final_write_file.loc[0:skip-1,'Q_Z']
    final_write_file.loc[0,'timestep'] = 0.0

    if train:
        if noise_level == 0:
            filename = 'train_clean_check.txt' 
        else:
            filename = f'train_noisy_v{noise_level}_check.txt' 
    else:
        filename = 'val_check.txt'  
    final_write_file.to_csv(os.path.join(dir, filename), 
                            header=True, 
                            index=None, 
                            sep=' ', 
                            mode='w')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Data Collection Script for AirSim')
    parser.add_argument('--dir', type=str)    
    parser.add_argument('--train', action='store_true',
                        help='Data Collection Mode')
    parser.add_argument('--imu_freq', type=int, default=100,
                        help='IMU sampling frequency')
    parser.add_argument('--img_freq', type=int, default=10,
                        help='Image sampling frequency, needs to be a factor of imu_freq')
    parser.add_argument('--noise_level', type=int, default=0,
                        help='Noise injected into training poses (used when imu-derived poses are generated)')
    parser.add_argument('--skip', type=int, default=5,
                        help='Spacing between frames used for relative poses (used when imu-derived poses are generated)')
    args = parser.parse_args()
    gen_imu_derived_rel_poses(args.dir, args.train, 
                              args.noise_level, args.skip, 
                              args.imu_freq, args.img_freq)