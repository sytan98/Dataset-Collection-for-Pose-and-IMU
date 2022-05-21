import airsim
import os
import time

def write_headers(file):
    file.writelines(
                'Vehicle_name\ttimestamp\tPOS_X\tPOS_Y\tPOS_Z\t'
                'Q_W\tQ_X\tQ_Y\tQ_Z\t'
                'LIN_ACC_X\tLIN_ACC_Y\tLIN_ACC_Z\t'
                'LIN_VEL_X\tLIN_VEL_Y\tLIN_VEL_Z\t'
                'ANG_VEL_X\tANG_VEL_Y\tANG_VEL_Z\t'
                'S_LIN_ACC_X\tS_LIN_ACC_Y\tS_LIN_ACC_Z\t'
                'S_ANG_VEL_X\tS_ANG_VEL_Y\tS_ANG_VEL_Z\t'
                'ImageFile\n'
                )
def record_instance(client, image_directory, index, file):
    client.simPause(True)
    # Get image
    response = client.simGetImage("0", airsim.ImageType.Scene)
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

    file.writelines( f' \t{timestamp}\t{pose.position.x_val:.8f}\t{pose.position.y_val:.8f}\t{pose.position.z_val:.8f}\t'
                    f'{pose.orientation.w_val:.8f}\t{pose.orientation.x_val:.8f}\t{pose.orientation.y_val:.8f}\t{pose.orientation.z_val:.8f}\t'
                    f'{kinematics.linear_acceleration.x_val:.8f}\t{kinematics.linear_acceleration.y_val:.8f}\t{kinematics.linear_acceleration.z_val:.8f}\t'
                    f'{kinematics.linear_velocity.x_val:.8f}\t{kinematics.linear_velocity.y_val:.8f}\t{kinematics.linear_velocity.z_val:.8f}\t'
                    f'{kinematics.angular_velocity.x_val:.8f}\t{kinematics.angular_velocity.y_val:.8f}\t{kinematics.angular_velocity.z_val:.8f}\t'
                    f'{imu_data.linear_acceleration.x_val:.8f}\t{imu_data.linear_acceleration.y_val:.8f}\t{imu_data.linear_acceleration.z_val:.8f}\t'
                    f'{imu_data.angular_velocity.x_val:.8f}\t{imu_data.angular_velocity.y_val:.8f}\t{imu_data.angular_velocity.z_val:.8f}\t'
                    f'{img_name}\n' )
    client.simPause(False)

def record_data(save_dir, recording_freq, simulation_clock, start_path_flag, finish_path_flag, finish_flag):
    time_period = 1/recording_freq
    print(f"Recording at {recording_freq}Hz (time period {time_period}s)")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()
    trajectory_index = 0		
    while not finish_flag.is_set():
        if start_path_flag.is_set():
            print("Start Recording")
            index = 0
            with open(os.path.join(save_dir, f"airsim_rec_{trajectory_index}.txt"), "w") as f:
                write_headers(f)
                image_directory = os.path.join(save_dir, f'images_{trajectory_index}')
                while not finish_path_flag.is_set():
                    record_instance(client, save_dir, image_directory, index)
                    time.sleep(time_period * simulation_clock)
                    index += 1
            trajectory_index += 1
            print("End recording")

