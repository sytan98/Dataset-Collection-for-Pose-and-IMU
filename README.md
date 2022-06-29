# Dataset-Collection-for-Pose-and-IMU 

## Purpose
This purpose of these scripts is to collect data for my FYP project in incorporating IMU data into camera pose regression models.
These scripts enable the collection of two types of datasets:
- Synthetic dataset from AirSim
- Real-world dataset collected from iPhone using ios_logger

## Repo Structure
- airsim_data_collection: Scripts used to collect and process synthetic dataset
	- [main.py](airsim_data_collection/main.py): main script to start airsim data collection.
	- [data_capture.py](airsim_data_collection/data_capture.py): define functions used to record data.
	- [drone.py](airsim_data_collection/drone.py): define functions used to navigate drone in airsim.
	- [generate_gt.py](airsim_data_collection/generate_gt.py): used after main script to process data to get imu-derived poses
	- [imu_derived_poses.ipynb](airsim_data_collection/imu_derived_poses.ipynb): Python notebooks to obtain statistics and plots
- real_world_data_collection: Scripts used to process real-world dataset
	- [video_to_frames.py](real_world_data_collection/video_to_frames.py): Convert video captured by ios logger to resized image frames.
	- [generate_colmap_geo_ref.py](real_world_data_collection/generate_colmap_geo_ref.py): obtain geo reference text file used in COLMAP model aligner.
	- [generate_gt.py](real_world_data_collection/generate_gt.py): generate imu derived poses based on ARKit, DeviceMotion and aligned COLMAP poses.
	- [colmap_processing.ipynb](real_world_data_collection/colmap_processing.ipynb): Python notebooks to obtain statistics and plots

- report_plots: Contains python notebooks used to plot diagrams
	- [generate_plots.ipynb](report_plots/generate_plots.ipynb): Python notebooks to obtain statistics and plots of experiments
	- [visualise_traj.ipynb](report_plots/visualise_traj.ipynb): Python notebooks to plot trajectories and cameras of camera pose predictions

## Main Dependencies
We use three separate environments, one for ```main.py``` and one for ```generate_gt.py``` and one for python notebooks. Use conda to create an environment according to environment.yml. 
```conda env create -f environment_{}.yml```

Environments:
- ```environment_airsim.yml``` 
This environment installs airsim and msgpack-rpc-python required for the Python API.
- ```environment_imu.yml``` 
This environment is needed to use for ```generate_gt.py``` as the airsim library has some conflicts with the AHRS library. 
An additional library set up for [AHRS](https://github.com/Mayitzin/ahrs) is required, simply pip installing AHRS is not enough as the latest version is required.
This environment is also used for python notebooks.

## airsim_data_collection
The scripts in here enable the collection of synchronised IMU sensor values and the cameras poses along with camera images in [AirSim](https://github.com/microsoft/AirSim).

- Multiprocessing is used to launch a process to control the movement of the rover and the other process is used to collect data.
- In order to collect synchronised data, the simulation is paused before every instance of data collection.

Ground Truth Format
Columns:
| ImageFile | timestep | POS_X | POS_Y | POS_Z | Q_W | Q_X | Q_Y | Q_Z | imu_POS_X | imu_POS_Y | imu_POS_Z | imu_Q_W |imu_Q_X | imu_Q_Y | imu_Q_Z |

IMU poses here are absolute poses, so camera pose regression model needs to calculate IMU derived relative poses by subtracting it with an original absolute pose of skip k frames before.
Each ground truth text is created for a specific skip value in MapNet. When skip value used in [IMUMapNet](https://github.com/sytan98/imumapnet) changes, the ground truth poses need to be changed.

### How to use
1. To record data on AirSim and run ```main.py```.
This can be used to collect for dataset for train/test/predefined path.
Example:
```
python .\main.py --save_dir "D:/Imperial/FYP/captured_data/airsim_drone_mode/building_10fps/check" --mode Train \
    --imu_freq 100 --img_freq 10 --xyz_dim -8 2 -8 0 -2 -4 --grid_res 4 4 2 2

```
2. Upon collecting the data, the next step will be to postprocess the IMU data to derive ground truth poses. This is done in ```generate_gt.py```. 
Example:
```
python .\generate_gt.py --dir "D:/Imperial/FYP/captured_data/airsim_drone_mode/building_new/train" \
    --train --imu_freq 100 --img_freq 10 --noise_level 2 --skip 5
```
3. ```imu_derived_poses.ipynb``` can be used to understand and view plots of the steps taken to process imu data.

## real_world_data_collection
The scripts in here are used to process the real-world dataset captured using an iPhone. It is meant to work alongside with ios_logger, a tool to collect synchronised IMU sensor data and image frames, and with COLMAP, an SfM tool to generate pseudo camera poses.

### How to use
1. Download and set up [ios_logger](https://github.com/Varvrar/ios_logger) app onto your iPhone. Capture the motion and use the ARKit mode to capture a trajectory. 

2. Use ```video_to_frames.py``` to split the video into single image frames that are resized for model training.

3. Use COLMAP to generate a sparse model from those frames. Shared intrinsic mode should be used.

4. Use ```generate_colmap_geo_ref.py``` to obtain the reference coordinates to be used in COLMAP model_aligner.
Example:
```
python .\generate_colmap_geo_ref.py --ios_logger_path "./data/2022-06-20T22-00-37" --output_txt 'geo_ref.txt'
```
5. Use the COLMAP model_aligner tool to align the reference frame to ARKit's reference frame.

6. Use ```generate.py``` to align all the data to the same reference frame (ARKit) and generate dataset for training.
Example:
```
python .\generate_gt.py --train --save_dir "D:/Imperial/FYP/captured_data/campus_v3/train/" \
    --colmap_path "C:/Users/tansi/Documents/Imperial_College_London/Year3/FYP_Project/aligned_campus_v3" \
    --ios_logger_path "./data/2022-06-20T22-00-37" --noise_level 1 --skip 15
```
## report_plots
This contains the python notebook used to generate plots in the report.