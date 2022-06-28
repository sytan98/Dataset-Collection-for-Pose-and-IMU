"""
Example:
python .\generate_colmap_geo_ref.py --ios_logger_path "./data/2022-06-20T22-00-37" --output_txt 'geo_ref.txt'
"""
import pandas as pd
import os
import numpy as np
import argparse


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generate geo referencing locations for COLMAP to align to ARKit poses')
    parser.add_argument('--ios_logger_path', type=str,
                        help='Text file of predefined path (used for debug mode)')
    parser.add_argument('--output_txt', type=str,
                        help='Text file to output')
    parser.add_argument('--frames', type=int, nargs=3,
                        help='Need at least 3 frames to align')
    args = parser.parse_args()


    df = pd.read_csv(os.path.join(args.ios_logger_path, 'ARposes.txt'), sep = ",",header=None)
    df.columns = ['time', 'POS_X', 'POS_Y', 'POS_Z', 'Q_W', 'Q_X', 'Q_Y', 'Q_Z']

    frames_df = pd.read_csv(os.path.join(args.ios_logger_path, 'Frames.txt'), sep = ",",header=None)
    frames_df.columns = ['time','frameNumber','focalLenghtX','focalLenghtY','principalPointX','principalPointY']
    frames_df["pos_x_interp"] = np.interp(frames_df["time"], df["time"], df["POS_X"])
    frames_df["pos_y_interp"] = np.interp(frames_df["time"], df["time"], df["POS_Y"])
    frames_df["pos_z_interp"] = np.interp(frames_df["time"], df["time"], df["POS_Z"])
    frames_df["q_w_interp"] = np.interp(frames_df["time"], df["time"], df["Q_W"])
    frames_df["q_x_interp"] = np.interp(frames_df["time"], df["time"], df["Q_X"])
    frames_df["q_y_interp"] = np.interp(frames_df["time"], df["time"], df["Q_Y"])
    frames_df["q_z_interp"] = np.interp(frames_df["time"], df["time"], df["Q_Z"])

    with open(args.output_txt, "w") as f:
        for idx,row in (frames_df[frames_df.frameNumber.isin(args.frames)][["pos_x_interp","pos_y_interp", "pos_z_interp"]]).iterrows():
            f.writelines(f'frame_{idx}.jpg {row.pos_x_interp} {row.pos_y_interp} {row.pos_z_interp}\n')