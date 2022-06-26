"""
Example:
python .\main.py --save_dir "D:/Imperial/FYP/captured_data/airsim_drone_mode/building_10fps/check" --mode Train \
    --imu_freq 100 --img_freq 10 --xyz_dim -8 2 -8 0 -2 -4 --grid_res 4 4 2 2
"""

from airsim_data_collection.process_imu import gen_imu_derived_rel_poses
import time
import os
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Data Colletion Script for AirSim')
    parser.add_argument('--mode', type=str, default='Train',
                        choices=('Train', 'Test', 'Debug'),
                        help='Data Collection Mode')
    parser.add_argument('--colmap_path', type=str)
    parser.add_argument('--ios_logger_path', type=str,
                        help='Text file of predefined path (used for debug mode)')
    parser.add_argument('--noise_level', type=int, default=0,
                        help='Noise injected into training poses (used when imu-derived poses are generated)')
    parser.add_argument('--skip', type=int, default=5,
                        help='Spacing between frames used for relative poses (used when imu-derived poses are generated)')
    args = parser.parse_args()


    aligned_colmap_image_file = 'C:/Users/tansi/Documents/Imperial_College_London/Year3/FYP_Project/aligned_campus_v3/images.txt'
    colmap_image_file = 'C:/Users/tansi/Documents/Imperial_College_London/Year3/FYP_Project/campus_colmap_v3/sparse/0/images.txt'
    motion_arh_data_file = './data/2022-06-20T22-00-37/MotARH.txt'
    motion_att_data_file = './data/2022-06-20T22-00-37/Motion.txt'
    frames_data_file = './data/2022-06-20T22-00-37/Frames.txt'
    arkit_data_file = './data/2022-06-20T22-00-37/ARposes.txt'
