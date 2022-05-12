from drone import Drone
from data_capture import record_data
import multiprocessing
import time

if __name__ == "__main__":
    # Building
    x_min, x_max = -8, 2
    y_min, y_max = -8, 0
    alt_min, alt_max = -2, -4
    # x_min, x_max = -4, 4
    # y_min, y_max = -8, 0
    # alt_min, alt_max = -2, -4
    res_x, res_y, res_z, res_yaw = 4, 4, 2, 2

    # Train dataset for building_99
    drone = Drone(x_min, x_max, y_min, y_max, alt_min, alt_max, res_x, res_y, res_z, res_yaw, speed=2)

    drone.connect_to_airsim()
    start_path_flag = multiprocessing.Event()
    finish_path_flag = multiprocessing.Event()
    finish_flag = multiprocessing.Event()

    save_dir = "D:/Imperial/FYP/captured_data/airsim_drone_mode/test"
    recording_freq = 1
    simulation_clock = 0.5
    data_recorder = multiprocessing.Process(
            name= "data recorder process", 
            target=record_data,
            args=(save_dir, recording_freq, simulation_clock, start_path_flag, finish_path_flag, finish_flag,))
    data_recorder.daemon = True
    data_recorder.start()
    time.sleep(5)
    drone.start_movement(start_path_flag, finish_path_flag, finish_flag)