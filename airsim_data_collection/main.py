"""
Example:
python .\main.py --save_dir "D:/Imperial/FYP/captured_data/airsim_drone_mode/building_10fps/check" --mode Train \
    --imu_freq 1 --img_freq 1 --xyz_dim -8 2 -8 0 -2 -4 --grid_res 4 4 2 2
"""

from drone import Drone
from data_capture import record_data
import multiprocessing
import time
import os
import argparse

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Data Colletion Script for AirSim')
    parser.add_argument('--save_dir', type=str)
    parser.add_argument('--mode', type=str, default='Train',
                        choices=('Train', 'Test', 'Debug'),
                        help='Data Collection Mode')
    parser.add_argument('--imu_freq', type=int, default=100,
                        help='IMU sampling frequency')
    parser.add_argument('--img_freq', type=int, default=10,
                        help='Image sampling frequency, needs to be a factor of imu_freq')
    parser.add_argument('--xyz_dim', type=int, nargs=6, help='min and max xyz dimensions')
    parser.add_argument('--grid_res', type=int, nargs=4, help="Grid resolution along x, y, z (in m) and yaw (in rad)")
    args = parser.parse_args()
    
    
    sim_clk = 0.5 #Taken from settings given to airsim
    x_min, x_max, y_min, y_max, alt_min, alt_max = args.xyz_dim
    res_x, res_y, res_z, res_yaw = args.grid_res

    # Train dataset for building_99
    drone = Drone(x_min, x_max, y_min, y_max, alt_min, alt_max, res_x, res_y, res_z, res_yaw, speed=2)

    drone.connect_to_airsim()
    start_path_flag = multiprocessing.Event()
    finish_path_flag = multiprocessing.Event()
    finish_flag = multiprocessing.Event()

    if not os.path.exists(args.save_dir):
        os.makedirs(args.save_dir)
    data_recorder = multiprocessing.Process(
                name= "data recorder process", 
                target=record_data,
                args=(args.save_dir, args.imu_freq, args.img_freq, sim_clk, start_path_flag, finish_path_flag, finish_flag,))
    data_recorder.daemon = True
    data_recorder.start()
    time.sleep(5)

    if args.mode == "Train": # Collect Training Set
        drone.start_grid_movement(start_path_flag, finish_path_flag, finish_flag)
    elif args.mode == "Test": # Collect Test Set
        drone.start_random_movement(start_path_flag, finish_path_flag, finish_flag)
    elif args.mode == "Debug": # Collect Debug
        drone.start_forward_movement(start_path_flag, finish_path_flag, finish_flag)