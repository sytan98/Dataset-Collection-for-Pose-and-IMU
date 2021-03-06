from multiprocessing.synchronize import Event
import airsim
import os

def write_headers(file) -> None:
    '''
    Write header titles for data text file 
    '''
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

def record_img(client: airsim.MultirotorClient, image_directory: str, index: int) -> None:
    '''
    Record an image frame and save to a directory
    '''

    response = client.simGetImage("0", airsim.ImageType.Scene)
    if not os.path.exists(image_directory):
        os.makedirs(image_directory)
    airsim.write_file(os.path.join(image_directory, f'{index}.png'), response)

def record_imu(client: airsim.MultirotorClient, index: int, file):
    '''
    Record an instance of IMU and kinematics data 
    '''
    img_name = f'{index}.png'

    # Get Pose
    pose = client.simGetCameraInfo("0").pose

    # Get Kinematics and imu
    kinematics = client.simGetGroundTruthKinematics()
    imu_data = client.getImuData()

    # Get timestamp (in ns)
    state = client.getMultirotorState()
    timestamp = state.timestamp

    file.writelines(f' \t{timestamp}\t{pose.position.x_val:.8f}\t{pose.position.y_val:.8f}\t{pose.position.z_val:.8f}\t'
                    f'{pose.orientation.w_val:.8f}\t{pose.orientation.x_val:.8f}\t{pose.orientation.y_val:.8f}\t{pose.orientation.z_val:.8f}\t'
                    f'{kinematics.linear_acceleration.x_val:.8f}\t{kinematics.linear_acceleration.y_val:.8f}\t{kinematics.linear_acceleration.z_val:.8f}\t'
                    f'{kinematics.linear_velocity.x_val:.8f}\t{kinematics.linear_velocity.y_val:.8f}\t{kinematics.linear_velocity.z_val:.8f}\t'
                    f'{kinematics.angular_velocity.x_val:.8f}\t{kinematics.angular_velocity.y_val:.8f}\t{kinematics.angular_velocity.z_val:.8f}\t'
                    f'{imu_data.linear_acceleration.x_val:.8f}\t{imu_data.linear_acceleration.y_val:.8f}\t{imu_data.linear_acceleration.z_val:.8f}\t'
                    f'{imu_data.angular_velocity.x_val:.8f}\t{imu_data.angular_velocity.y_val:.8f}\t{imu_data.angular_velocity.z_val:.8f}\t'
                    f'{img_name}\n' )

def record_data(save_dir: str, imu_recording_freq: int, img_recording_freq: int, 
                sim_clk: float, start_path_flag: Event, finish_path_flag: Event, finish_flag: Event):
    '''
    Data Capture process which checks if time period for sampling has passed
    '''
    imu_time_period = 1/imu_recording_freq
    img_time_period = 1/img_recording_freq
    print(f"Recording imu at {imu_recording_freq}Hz (time period {imu_time_period}s)")
    print(f"Recording images at {img_recording_freq}Hz (time period {img_time_period}s)")
    img_time_scale = int(img_time_period / imu_time_period)
    print(f"Image saved for every {img_time_scale} imu recording")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    client.enableApiControl(True)
    client.armDisarm(True)
    client.takeoffAsync().join()	
    while not finish_flag.is_set():
        if start_path_flag.is_set():
            print("Start Recording")
            index = 0
            
            with open(os.path.join(save_dir, f"airsim_rec.txt"), "w") as f:
                write_headers(f)
                image_directory = os.path.join(save_dir, f'images')
                client.simPause(True)
                last = client.getMultirotorState().timestamp
                while not finish_path_flag.is_set():
                    now = client.getMultirotorState().timestamp
                    time_taken = float(now - last) / 10 ** 9
                    if time_taken > imu_time_period:
                        last = now
                        record_imu(client, index, f)
                        if index % img_time_scale == 0:
                            record_img(client, image_directory, index)
                        index += 1
                    # Uses CPU clock so needs to divide by sim clock
                    # TODO: sometimes real sim_clk differs from sim clock set. Might be better to get the real the sim clock instead
                    client.simContinueForTime(imu_time_period / sim_clk / 4) 
            print("End recording")

