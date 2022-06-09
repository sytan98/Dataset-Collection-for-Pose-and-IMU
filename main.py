from drone import Drone
from data_capture import record_data
import multiprocessing
import time
import os

if __name__ == "__main__":
    mode = 2
    imu_recording_freq = 100
    img_recording_freq = 10
    sim_clk = 0.5
    # Building
    x_min, x_max = -8, 2
    y_min, y_max = -8, 0
    alt_min, alt_max = -2, -4
    res_x, res_y, res_z, res_yaw = 4, 4, 2, 2

    # Train dataset for building_99
    drone = Drone(x_min, x_max, y_min, y_max, alt_min, alt_max, res_x, res_y, res_z, res_yaw, speed=2)

    drone.connect_to_airsim()
    start_path_flag = multiprocessing.Event()
    finish_path_flag = multiprocessing.Event()
    finish_flag = multiprocessing.Event()

    save_dir = "D:/Imperial/FYP/captured_data/airsim_drone_mode/building_10fps/val"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    data_recorder = multiprocessing.Process(
                name= "data recorder process", 
                target=record_data,
                args=(save_dir, imu_recording_freq, img_recording_freq, sim_clk, start_path_flag, finish_path_flag, finish_flag,))
    data_recorder.daemon = True
    data_recorder.start()
    time.sleep(5)

    if mode == 1: # Collect Training Set
        drone.start_grid_movement(start_path_flag, finish_path_flag, finish_flag)
    elif mode == 2: # Collect Test Set
        drone.start_random_movement(start_path_flag, finish_path_flag, finish_flag)
    elif mode == 3: # Collect Debug
        drone.start_forward_movement(start_path_flag, finish_path_flag, finish_flag)