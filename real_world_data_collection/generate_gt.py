"""
Example:
python .\generate_gt.py --train --save_dir "D:/Imperial/FYP/captured_data/campus_v3/train/" \
    --colmap_path "C:/Users/tansi/Documents/Imperial_College_London/Year3/FYP_Project/aligned_campus_v3" \
    --ios_logger_path "./data/2022-06-20T22-00-37" --noise_level 1 --skip 15
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import rowan
from scipy.spatial.transform import Rotation as R
from scipy import integrate
from scipy import signal
import sys
sys.path.append("../")
from utils import convert_body_to_world

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Data Collection Script for real-world datasets')
    parser.add_argument('--train', action='store_true',
                        help='Data Collection Mode')
    parser.add_argument('--save_dir', type=str, 
                        help='Folder to save to')
    parser.add_argument('--colmap_path', type=str, 
                        help='Folder where COLMAP project has been exported as text to')
    parser.add_argument('--ios_logger_path', type=str,
                        help='Text file of predefined path (used for debug mode)')
    parser.add_argument('--noise_level', type=int, default=0,
                        help='Noise injected into training poses (used when imu-derived poses are generated)')
    parser.add_argument('--skip', type=int, default=15,
                        help='Spacing between frames used for relative poses (used when imu-derived poses are generated)')
    args = parser.parse_args()

    # load Camera Frames data
    frames_data_file = os.path.join(args.ios_logger_path, 'Frames.txt')
    frames_df = pd.read_csv(frames_data_file, sep = ",",header=None)
    frames_df.columns = ['time','frameNumber','focalLenghtX','focalLenghtY','principalPointX','principalPointY']

    # load DeviceMotion imu acceleration data
    motion_arh_data_file = os.path.join(args.ios_logger_path, 'MotARH.txt')
    motion_arh_df = pd.read_csv(motion_arh_data_file, sep = ",",header=None)
    motion_arh_df.columns = ['time','ang_vel_x','ang_vel_y','ang_vel_z',
                            'gravity_x','gravity_y','gravity_z',
                            'imu_acc_x','imu_acc_y','imu_acc_z', 'motionHeading']

    # load DeviceMotion attitude data
    motion_att_data_file = os.path.join(args.ios_logger_path, 'Motion.txt')
    orientation = pd.read_csv(motion_att_data_file, sep = ",",header=None)
    orientation.columns = ['time','imu_Q_W', 'imu_Q_X', 'imu_Q_Y', 'imu_Q_Z']

    print("--------Aligning Image frames, IMU, COLMAP poses---------")
    # Align image frames and imu sensors
    frames_df["imu_acc_x"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["imu_acc_x"])
    frames_df["imu_acc_y"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["imu_acc_y"])
    frames_df["imu_acc_z"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["imu_acc_z"])
    frames_df["ang_vel_x"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["ang_vel_x"])
    frames_df["ang_vel_y"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["ang_vel_y"])
    frames_df["ang_vel_z"] = np.interp(frames_df["time"], motion_arh_df["time"], motion_arh_df["ang_vel_z"])
    frames_df["imu_Q_W"] = np.interp(frames_df["time"], orientation["time"], orientation["imu_Q_W"])
    frames_df["imu_Q_X"] = np.interp(frames_df["time"], orientation["time"], orientation["imu_Q_X"])
    frames_df["imu_Q_Y"] = np.interp(frames_df["time"], orientation["time"], orientation["imu_Q_Y"])
    frames_df["imu_Q_Z"] = np.interp(frames_df["time"], orientation["time"], orientation["imu_Q_Z"])

    frames_df["imu_acc_x_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['imu_acc_x', 'imu_acc_y', 'imu_acc_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[0], axis = 1)
    frames_df["imu_acc_y_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['imu_acc_x', 'imu_acc_y', 'imu_acc_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[1], axis = 1)
    frames_df["imu_acc_z_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['imu_acc_x', 'imu_acc_y', 'imu_acc_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[2], axis = 1)
    frames_df["ang_vel_x_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['ang_vel_x', 'ang_vel_y', 'ang_vel_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[0], axis = 1)
    frames_df["ang_vel_y_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['ang_vel_x', 'ang_vel_y', 'ang_vel_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[1], axis = 1)
    frames_df["ang_vel_z_global"]  = frames_df.apply(lambda row :  convert_body_to_world(
                                            np.array(row[['ang_vel_x', 'ang_vel_y', 'ang_vel_z']]),
                                            np.array(row[['imu_Q_X', 'imu_Q_Y', 'imu_Q_Z', 'imu_Q_W', ]]))[2], axis = 1)

    frames_df["seconds"] = frames_df["time"] - frames_df["time"][0]
    frames_df["timestep"] = frames_df["time"] - frames_df["time"].shift(1)
    frames_df["lin_vel_x"] = integrate.cumtrapz(frames_df["imu_acc_x_global"]*-9.81, frames_df["seconds"], initial=0)
    frames_df["lin_vel_y"] = integrate.cumtrapz(frames_df["imu_acc_y_global"]*-9.81, frames_df["seconds"], initial=0)
    frames_df["lin_vel_z"] = integrate.cumtrapz(frames_df["imu_acc_z_global"]*-9.81, frames_df["seconds"], initial=0)
    frames_df["lin_vel_x"] = signal.detrend(frames_df["lin_vel_x"])
    frames_df["lin_vel_y"] = signal.detrend(frames_df["lin_vel_y"])
    frames_df["lin_vel_z"] = signal.detrend(frames_df["lin_vel_z"])
    frames_df["prev_lin_vel_x"] = frames_df["lin_vel_x"].shift(1)
    frames_df["prev_lin_vel_y"] = frames_df["lin_vel_y"].shift(1)
    frames_df["prev_lin_vel_z"] = frames_df["lin_vel_z"].shift(1)
    frames_df["prev_imu_acc_x"] = frames_df["imu_acc_x"].shift(1)
    frames_df["prev_imu_acc_y"] = frames_df["imu_acc_y"].shift(1)
    frames_df["prev_imu_acc_z"] = frames_df["imu_acc_z"].shift(1)

    # Load COLMAP camera poses
    aligned_colmap_image_file = os.path.join(args.colmap_path,'images.txt')
    camera_poses = pd.DataFrame()
    with open(aligned_colmap_image_file, "r") as f:
        for line in f.readlines()[4::2]:
            values = line.strip('\n').split(" ")
            name = values[-1]
            q_w, q_x, q_y, q_z = values[1:5]
            r = R.from_quat(np.hstack([float(q_x), float(q_y), float(q_z), float(q_w)]))
            inv_r = r.inv()
            t_x, t_y, t_z = values[5:8]
            t_x, t_y, t_z = float(t_x), float(t_y), float(t_z)
            x, y, z = -inv_r.apply(np.hstack([t_x, t_y, t_z]))
            q_x_invert, q_y_invert, q_z_invert, q_w_invert = inv_r.as_quat()
            camera_poses = pd.concat([camera_poses, 
                                    pd.DataFrame.from_records([{'ImageFile': name, 
                                                                'POS_X': x,
                                                                'POS_Y': y,
                                                                'POS_Z': z, 
                                                                'Q_W': q_w_invert,
                                                                'Q_X': q_x_invert,
                                                                'Q_Y': q_y_invert,
                                                                'Q_Z': q_z_invert}])], ignore_index=True)
    camera_poses["idx"] = camera_poses.apply(lambda row: int(row["ImageFile"].split(".")[0][6:]), axis = 1)
    camera_poses.sort_values(by="idx", inplace= True)
    camera_poses.reset_index(drop=True, inplace=True)
    camera_poses.Q_W = camera_poses.Q_W.astype(float)
    camera_poses.Q_X = camera_poses.Q_X.astype(float)
    camera_poses.Q_Y = camera_poses.Q_Y.astype(float)
    camera_poses.Q_Z = camera_poses.Q_Z.astype(float)   

    temp = pd.DataFrame()
    temp["Q_X"] = camera_poses["Q_W"]
    temp["Q_Y"] = camera_poses["Q_Z"]
    temp["Q_Z"] = -camera_poses["Q_Y"]
    temp["Q_W"] = -camera_poses["Q_X"]

    camera_poses["Q_X"] = temp["Q_X"]
    camera_poses["Q_Y"] = temp["Q_Y"]
    camera_poses["Q_Z"] = temp["Q_Z"]
    camera_poses["Q_W"] = temp["Q_W"]

    # Inject Noise
    np.random.seed(123)
    if args.noise_level == 0 or not args.train:
        noise_x = 0
        noise_y = 0
        noise_z = 0
        noise_qw = 0
        noise_qx = 0
        noise_qy = 0
        noise_qz = 0
    elif args.noise_level == 1:
        noise_x = np.random.normal(0,0.1,len(camera_poses))
        noise_y = np.random.normal(0,0.05,len(camera_poses))
        noise_z = np.random.normal(0,0.1,len(camera_poses))
        noise_qw = np.random.normal(0,0.03,len(camera_poses))
        noise_qx = np.random.normal(0,0.03,len(camera_poses))
        noise_qy = np.random.normal(0,0.03,len(camera_poses))
        noise_qz = np.random.normal(0,0.03,len(camera_poses))

    camera_poses["noisy_POS_X"] = noise_x + camera_poses["POS_X"]
    camera_poses["noisy_POS_Y"] = noise_y + camera_poses["POS_Y"]
    camera_poses["noisy_POS_Z"] = noise_z + camera_poses["POS_Z"]
    camera_poses["noisy_Q_W"] = noise_qw + camera_poses["Q_W"]
    camera_poses["noisy_Q_X"] = noise_qx + camera_poses["Q_X"]
    camera_poses["noisy_Q_Y"] = noise_qy + camera_poses["Q_Y"]
    camera_poses["noisy_Q_Z"] = noise_qz + camera_poses["Q_Z"]

    # Ensure that noisy quaternions is a valid rotation
    normalized_q = pd.DataFrame(rowan.normalize(camera_poses[["noisy_Q_W", "noisy_Q_X", "noisy_Q_Y", "noisy_Q_Z"]]), columns=["noisy_Q_W", "noisy_Q_X", "noisy_Q_Y", "noisy_Q_Z"])
    camera_poses["noisy_Q_W"] = normalized_q["noisy_Q_W"]
    camera_poses["noisy_Q_X"] = normalized_q["noisy_Q_X"]
    camera_poses["noisy_Q_Y"] = normalized_q["noisy_Q_Y"]
    camera_poses["noisy_Q_Z"] = normalized_q["noisy_Q_Z"]

    imu_colmap_df = pd.concat([frames_df, camera_poses], axis = 1)
    k = args.skip

    print("----------Getting relative translation from IMU----------") 
    imu_rel_trans = []
    for i in range(k, len(imu_colmap_df)):
        row = imu_colmap_df.iloc[i,:]
        time = row.time
        x, y, z = row.noisy_POS_X, row.noisy_POS_Y, row.noisy_POS_Z
        imu_idx = 1
        while imu_colmap_df.iloc[i - k + imu_idx,0] < time:
            imu_row = imu_colmap_df.iloc[i - k + imu_idx,:]
            z -= imu_row.prev_lin_vel_x * imu_row.timestep + 1/2 * imu_row.prev_imu_acc_x * imu_row.timestep ** 2 
            x -= imu_row.prev_lin_vel_y * imu_row.timestep + 1/2 * imu_row.prev_imu_acc_y * imu_row.timestep ** 2 
            y += imu_row.prev_lin_vel_z * imu_row.timestep + 1/2 * imu_row.prev_imu_acc_z * imu_row.timestep ** 2
            imu_idx += 1
        imu_rel_trans.append([x, y, z])

    imu_trans = pd.DataFrame(imu_rel_trans, columns=["imu_POS_X", "imu_POS_Y","imu_POS_Z"])
    imu_trans["idx"] = [i for i in range(k, len(imu_colmap_df))]
    imu_trans.set_index("idx", inplace=True)

    imu_colmap_df['prev_ang_vel_x_global'] = imu_colmap_df['ang_vel_x_global'].shift(1) 
    imu_colmap_df['prev_ang_vel_y_global'] = imu_colmap_df['ang_vel_y_global'].shift(1)
    imu_colmap_df['prev_ang_vel_z_global'] = imu_colmap_df['ang_vel_z_global'].shift(1)

    print("----------Getting relative orientation from IMU----------")
    abs_results = []
    for i in range(k, len(imu_colmap_df)):
        row = imu_colmap_df.iloc[i,:]
        time = row.time
        qw = row.noisy_Q_W
        qx = row.noisy_Q_X
        qy = row.noisy_Q_Y
        qz = row.noisy_Q_Z
        q_prev = np.array([qw, qx, qy, qz])
        cur_q = q_prev
        imu_idx = 1 
        while imu_colmap_df.iloc[i - k + imu_idx,0] < time:
            imu_row = imu_colmap_df.iloc[i - k + imu_idx,:]
            cur_q = rowan.calculus.integrate (cur_q, 
                                            np.hstack([-row["prev_ang_vel_y_global"],
                                                    row["prev_ang_vel_z_global"], 
                                                    -row["prev_ang_vel_x_global"]]),
                                            row['timestep'])
            cur_q = rowan.normalize(cur_q)
            imu_idx += 1
        abs_results.append(cur_q)

    imu_q_rowan = pd.DataFrame(abs_results, columns=["arkit_imu_Q_W", "arkit_imu_Q_X", "arkit_imu_Q_Y", "arkit_imu_Q_Z"])
    imu_q_rowan["idx"] = [i for i in range(k, len(imu_colmap_df))]
    imu_q_rowan.set_index("idx", inplace=True)

    imu_colmap_df = pd.concat([imu_colmap_df[["ImageFile", "timestep", "POS_X", "POS_Y", "POS_Z", "Q_W", "Q_X", "Q_Y", "Q_Z"]], imu_trans, imu_q_rowan], axis = 1)

    final_write_file = imu_colmap_df[["ImageFile",
                                     "POS_X", "POS_Y", "POS_Z", 
                                     "Q_W", "Q_X", "Q_Y", "Q_Z",
                                     "imu_POS_X", "imu_POS_Y", "imu_POS_Z", 
                                     "arkit_imu_Q_W", "arkit_imu_Q_X","arkit_imu_Q_Y","arkit_imu_Q_Z"
                                    ]]
    final_write_file.rename(columns={"arkit_imu_Q_W": "imu_Q_W", 
                                    "arkit_imu_Q_X": "imu_Q_X",
                                    "arkit_imu_Q_Y": "imu_Q_Y",
                                    "arkit_imu_Q_Z": "imu_Q_Z"}, inplace=True)
    final_write_file.loc[0:k-1,'imu_POS_X'] = final_write_file.loc[0:k-1,'POS_X']
    final_write_file.loc[0:k-1,'imu_POS_Y'] = final_write_file.loc[0:k-1,'POS_Y']
    final_write_file.loc[0:k-1,'imu_POS_Z'] = final_write_file.loc[0:k-1,'POS_Z']
    final_write_file.loc[0:k-1,'imu_Q_W'] = final_write_file.loc[0:k-1,'Q_W']
    final_write_file.loc[0:k-1,'imu_Q_X'] = final_write_file.loc[0:k-1,'Q_X']
    final_write_file.loc[0:k-1,'imu_Q_Y'] = final_write_file.loc[0:k-1,'Q_Y']
    final_write_file.loc[0:k-1,'imu_Q_Z'] = final_write_file.loc[0:k-1,'Q_Z']

    if args.train:
        if args.noise_level == 0:
            filename = 'train_clean' 
        else:
            filename = f'train_noisy_v{args.noise_level}' 
        filename += f'_skip_{args.skip}.txt'
    else:
        filename = 'val.txt'  
    final_write_file.to_csv(os.path.join(args.save_dir, filename), 
                            header=True, 
                            index=None, 
                            sep=' ', 
                            mode='w')