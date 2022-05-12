import airsim
import os
import time

def record_data(start_path_flag, finish_path_flag, finish_flag):
	client = airsim.MultirotorClient()
	client.confirmConnection()
	client.enableApiControl(True)
	client.armDisarm(True)
	client.takeoffAsync().join()
	client.moveToZAsync(2, -2).join()
	tmp_dir = "D:/Imperial/FYP/captured_data/airsim_drone_mode/test"
	trajectory_index = 0		
	while not finish_flag.is_set():
		if start_path_flag.is_set():
			index = 0
			with open(os.path.join(tmp_dir, f"airsim_rec_{trajectory_index}.txt"), "w") as f:
				f.writelines(
						'Vehicle_name\ttimestamp\tPOS_X\tPOS_Y\tPOS_Z\t'
						'Q_W\tQ_X\tQ_Y\tQ_Z\t'
						'LIN_ACC_X\tLIN_ACC_Y\tLIN_ACC_Z\t'
						'LIN_VEL_X\tLIN_VEL_Y\tLIN_VEL_Z\t'
						'ANG_VEL_X\tANG_VEL_Y\tANG_VEL_Z\t'
						'S_LIN_ACC_X\tS_LIN_ACC_Y\tS_LIN_ACC_Z\t'
						'S_ANG_VEL_X\tS_ANG_VEL_Y\tS_ANG_VEL_Z\t'
						'ImageFile\n'
						)
				while not finish_path_flag.is_set():
					client.simPause(True)

					# Get image
					response = client.simGetImage("0", airsim.ImageType.Scene)
					image_directory = os.path.join(tmp_dir, f'images_{trajectory_index}')
					if not os.path.exists(image_directory):
						os.makedirs(image_directory)
					airsim.write_file(os.path.join(image_directory, f'{index}.png'), response)
					img_name = f'{index}.png'

					# Get Pose
					pose = client.simGetVehiclePose()
					# print(current_pose)

					# Get Kinematics and imu
					kinematics = client.simGetGroundTruthKinematics()
					imu_data = client.getImuData()

					# Get timestamp (in ns)
					state = client.getMultirotorState()
					# print(datetime.fromtimestamp(state.timestamp/1e9))
					timestamp = state.timestamp

					f.writelines( f' \t{timestamp}\t{pose.position.x_val:.8f}\t{pose.position.y_val:.8f}\t{pose.position.z_val:.8f}\t'
									f'{pose.orientation.w_val:.8f}\t{pose.orientation.x_val:.8f}\t{pose.orientation.y_val:.8f}\t{pose.orientation.z_val:.8f}\t'
									f'{kinematics.linear_acceleration.x_val:.8f}\t{kinematics.linear_acceleration.y_val:.8f}\t{kinematics.linear_acceleration.z_val:.8f}\t'
									f'{kinematics.linear_velocity.x_val:.8f}\t{kinematics.linear_velocity.y_val:.8f}\t{kinematics.linear_velocity.z_val:.8f}\t'
									f'{kinematics.angular_velocity.x_val:.8f}\t{kinematics.angular_velocity.y_val:.8f}\t{kinematics.angular_velocity.z_val:.8f}\t'
									f'{imu_data.linear_acceleration.x_val:.8f}\t{imu_data.linear_acceleration.y_val:.8f}\t{imu_data.linear_acceleration.z_val:.8f}\t'
									f'{imu_data.angular_velocity.x_val:.8f}\t{imu_data.angular_velocity.y_val:.8f}\t{imu_data.angular_velocity.z_val:.8f}\t'
									f'{img_name}\n' )
					client.simPause(False)
					time.sleep(2)
					index += 1
			trajectory_index += 1

