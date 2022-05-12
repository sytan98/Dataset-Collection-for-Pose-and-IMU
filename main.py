from drone import Drone
from data_capture import record_data
import multiprocessing
import time

if __name__ == "__main__":
	# Building
	x_min, x_max = -8, 2
	y_min, y_max = -10, 0
	alt_min, alt_max = -2, -4
	# x_min, x_max = -4, 4
	# y_min, y_max = -8, 0
	# alt_min, alt_max = -2, -4
	res_x, res_y, res_z, res_yaw = 4, 4, 2, 2

	# Train dataset for building_99
	data_capture_drone = Drone(x_min, x_max, y_min, y_max, alt_min, alt_max, res_x, res_y, res_z, res_yaw, speed=2)

	data_capture_drone.connect_to_airsim()
	start_path_flag = multiprocessing.Event()
	finish_path_flag = multiprocessing.Event()
	finish_flag = multiprocessing.Event()

	data_recorder = multiprocessing.Process(
			name= "data recorder process", 
			target=record_data,
			args=(start_path_flag, finish_path_flag, finish_flag,))
	data_recorder.daemon = True
	data_recorder.start()
	time.sleep(5)
	data_capture_drone.start_movement(start_path_flag, finish_path_flag, finish_flag)