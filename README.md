# Dataset-Collection-for-Pose-and-IMU

## Purpose
This purpose of these scripts is to collect data for my FYP project in incorporating IMU data into camera pose regression models.
These scripts enable the collection of two types of datasets:
- Synthetic dataset from AirSim
- Real-world dataset collected from iPhone using ios_logger

## airsim_data_collection
The scripts in here enable the collection of synchronised IMU sensor values and the cameras poses along with camera images in [AirSim](https://github.com/microsoft/AirSim).

- Multiprocessing is used to launch a process to control the movement of the rover and the other process is used to collect data.
- In order to collect synchronised data, the simulation is paused before every instance of data collection.

### How to use
Use conda to create an environment according to environment.yml. 
```conda env create -f environment.yml```
This installs the libraries required to use AirSim's Python API and run ```main.py```.

Upon collecting the data, the next step will be to postprocess the IMU data to derive ground truth poses. This is done in ```imu_derived_poses.ipynb```. Use a different kernel to run the notebook, as airsim python api has some conflicting dependencies with tornado as iPython notebooks uses a newer version of tornado. An additional library set up for [AHRS](https://github.com/Mayitzin/ahrs) is required, simply pip installing AHRS is not enough as the latest version is required.

## real_world_data_collection
The scripts in here are used to process the real-world dataset captured using an iPhone. It is meant to work alongside with ios_logger, a tool to collect synchronised IMU sensor data and image frames, and with COLMAP, an SfM tool to generate pseudo camera poses.

### How to use
Download and set up [ios_logger](https://github.com/Varvrar/ios_logger) app onto your iPhone. Capture the motion and use the ARKit mode to capture a trajectory. 

Use ```video_to_frames.py``` to split the video into single image frames that are resized for model training.

Use COLMAP to generate a sparse model from those frames. Shared intrinsic mode should be used.

Use ```iphone_data_processing.ipynb``` to check the alignment of the DeviceMotion reference frame and the ARKit reference frame. Also use this script to obtain the reference coordinates to be used in COLMAP model_aligner.

Use the COLMAP model_aligner tool to align the reference frame to ARKit's reference frame.

Use ```colmap_processing.ipynb``` to align all the data to the same reference frame (ARKit) and generate dataset for training.

## report_plots
This contains the python notebook used to generate plots in the report.