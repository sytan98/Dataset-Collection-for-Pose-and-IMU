# AirSim-Dataset-Collection-for-Pose-and-IMU

## Purpose
This purpose of these scripts is to collect data for my FYP project in incorporating IMU data into camera pose regression models.
These scripts enable the collection of synchronised IMU sensor values and the cameras poses along with camera images in AirSim.

- Multiprocessing is used to launch a process to control the movement of the rover and the other process is used to collect data.
- In order to collect synchronised data, the simulation is paused before every instance of data collection.

## How to use
Use conda to create an environment according to environment.yml
```conda env create -f environment.yml```

Use a different kernel to run the ```data_vis_processing.ipynb```, as airsim python api has some conflicting dependencies with tornado as iPython notebooks uses a newer version of tornado.