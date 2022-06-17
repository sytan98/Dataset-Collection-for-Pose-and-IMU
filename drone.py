import os
import airsim
import numpy as np
import pprint
import math
import random
import time
from itertools import product
# from data_capture import record_instance, write_headers
from utils import flatten, has_items
random.seed(123)

def get_trajectory_and_update_grid(total_grid, x_vals, y_vals, z_vals):
    is_start_point_found = False
    while not is_start_point_found:
        start_x, start_y, start_z = random.randint(0, len(x_vals)-1), random.randint(0, len(y_vals)-1), random.randint(0, len(z_vals)-1)
        if len(total_grid[start_x][start_y][start_z]) > 0:
            is_start_point_found = True
    path = []
    queue = [(start_x, start_y, start_z)]
    while queue:
        x, y, z = queue.pop(0)
        if len(total_grid[x][y][z]) > 0:
            path.append(total_grid[x][y][z][0])
            del total_grid[x][y][z][0]

            all_candidates = list(product([x - 1, x, x +1],[y - 1, y, y + 1],[z - 1, z, z + 1]))
            valid_candidates = []
            for i in all_candidates:
                x, y, z = i
                if x >= 0 and x < len(x_vals) and \
                   y >= 0 and y < len(y_vals) and \
                   z >= 0 and z < len(z_vals) and \
                   len(total_grid[x][y][z]) > 0:
                    valid_candidates.append((x, y, z))
            
            if valid_candidates:
                queue.append(random.sample(valid_candidates, k= 1)[0])
            
    return path


class Drone:
    def __init__(self,  x_min, x_max, y_min, y_max, alt_min, alt_max, res_x=4, res_y=4, res_z=2, res_yaw=2, speed=2) -> None:
        self.speed = speed
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.alt_min = alt_min
        self.alt_max = alt_max
        self.x_vals = np.arange(x_min, x_max + 1, res_x)
        self.y_vals = np.arange(y_min, y_max + 1, res_y)
        self.z_vals = np.arange(alt_max, alt_min + 1, res_z)
        self.yaw_vals = np.arange(0, int(2*math.pi), res_yaw)
        print("x vals", self.x_vals)
        print("y vals", self.y_vals)
        print("z vals", self.z_vals)
        print("yaw vals", self.yaw_vals)

    def connect_to_airsim(self):
        # Connect to AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    # Get Drone state and IMU Data
    def get_data(self):
        state = self.client.getMultirotorState()
        print(f"state: {pprint.pformat(state)}")

        imu_data = self.client.getImuData()
        print(f"state: {pprint.pformat(imu_data)}")

    # Take off
    def take_off(self):
        airsim.wait_key("Press any key for takeoff")
        print("Taking off ...")
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()

    def land(self):
        print("landing...")
        self.client.reset()
        print("disarming...")
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        print("done.")

    def start_grid_movement(self, start_path_flag, finish_path_flag, finish_flag):
        num_of_points = 0
        total_grid = [[[None for _ in self.z_vals] for _ in self.y_vals] for _ in self.x_vals]
        for i, x in enumerate(self.x_vals):
            for j, y in enumerate(self.y_vals):
                for k, z in enumerate(self.z_vals):
                    random_start_yaw = random.random() * 2*math.pi
                    points = []
                    for yaw_shift in self.yaw_vals:
                        yaw = random_start_yaw + yaw_shift
                        yaw = yaw if yaw < 2*math.pi else yaw - 2*math.pi
                        print(f"x: {x}, y: {y}, z: {z}, yaw: {math.degrees(yaw)} deg")
                        points.append((x, y, z, int(math.degrees(yaw))))
                        num_of_points += 1
                    total_grid[i][j][k] = points
        paths = []
        
        while has_items(flatten(total_grid)):
            path = get_trajectory_and_update_grid(total_grid, self.x_vals, self.y_vals, self.z_vals)
            paths.append(path)

        print(paths)
        print("Path lengths")
        for path in paths:
            print(f"length of path = {len(path)}")
        print(f"remaining ({len(total_grid)}) {total_grid}")
        print(f"total number = {num_of_points}")
        
        for path in paths:
            if len(path) > 50:
                print("Starting new path")
                x, y, z, yaw = path[0]
                self.client.simSetVehiclePose(airsim.Pose(
                                    airsim.Vector3r(float(x), float(y), float(z)), airsim.to_quaternion(0, 0, float(yaw))), True)
                time.sleep(1)
                finish_path_flag.clear()
                start_path_flag.set()
                for waypoint in path:
                    print(waypoint)
                    x, y, z, yaw = waypoint
                    self.client.moveToPositionAsync(float(x), float(y), float(z), self.speed, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode(False, float(yaw))).join()
                start_path_flag.clear()
                finish_path_flag.set()
                time.sleep(2)
                break

        time.sleep(2)
        finish_flag.set()
        self.land()

    def start_random_movement(self, start_path_flag, finish_path_flag, finish_flag):
        def get_random_x_y_alt():
            return random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max), random.uniform(self.alt_min, self.alt_max)

        random_x_y_alt = [get_random_x_y_alt() for i in range(20)]
        print(random_x_y_alt)
        path = [airsim.Vector3r(*point) for point in random_x_y_alt]
        self.client.simSetVehiclePose(airsim.Pose(path[0], airsim.to_quaternion(0, 0, 0)), True)
        time.sleep(1)
        finish_path_flag.clear()
        start_path_flag.set()
        self.client.moveOnPathAsync(path, self.speed, 120,
                                    airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False, 0)).join()
        start_path_flag.clear()
        finish_path_flag.set()
        
        time.sleep(2)
        finish_flag.set()
        self.land()

    def start_forward_movement(self, start_path_flag, finish_path_flag, finish_flag):
        train_path = [airsim.Vector3r(0, 0, -3), 
                      airsim.Vector3r(-4, 0, -3), 
                      airsim.Vector3r(-4, 0, -6), 
                      airsim.Vector3r(-4, 0, -3), 
                      airsim.Vector3r(-8, 0, -3), 
                      airsim.Vector3r(-8, 0, -6), 
                      airsim.Vector3r(0, 0, -6),
                      airsim.Vector3r(-8, 0, -3)]

        test_path = [airsim.Vector3r(-1, -3, -2), 
                     airsim.Vector3r(-4, -3, -2),
                     airsim.Vector3r(-4, -3, -7),
                     airsim.Vector3r(-8, -3, -7),]

        path = test_path
        self.client.simSetVehiclePose(airsim.Pose(path[0], airsim.to_quaternion(0, 0, math.radians(270))), True)
        time.sleep(5)
        finish_path_flag.clear()
        start_path_flag.set()
        self.client.moveOnPathAsync(path, self.speed, 120,
                                    airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 270)).join()
        start_path_flag.clear()
        finish_path_flag.set()
        
        time.sleep(2)
        finish_flag.set()
        self.land()