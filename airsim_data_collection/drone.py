from multiprocessing.synchronize import Event
import airsim
import numpy as np
import math
import random
import time
from itertools import product
from typing import Iterator, List, Tuple 
random.seed(123)

def flatten_nested_list(input_list: List):
    '''
    Recursively iterate through values in nested lists.
    '''
    for item in input_list:
        if isinstance(item, list):
            for child_item in flatten_nested_list(item):
                yield child_item
        else:
            yield item


def has_items(seq : Iterator):
    '''
    Checks if an iterator has any items.
    '''
    return any(1 for _ in seq)

def generate_grid(x_vals, y_vals, z_vals, yaw_vals) -> List[List[List]]:
    '''
    Generate a grid of waypoints
    '''
    total_grid = [[[None for _ in z_vals]
                    for _ in y_vals] for _ in x_vals]
    for i, x in enumerate(x_vals):
        for j, y in enumerate(y_vals):
            for k, z in enumerate(z_vals):
                random_start_yaw = random.random() * 2*math.pi
                points = []
                for yaw_shift in yaw_vals:
                    yaw = random_start_yaw + yaw_shift
                    yaw = yaw if yaw < 2*math.pi else yaw - 2*math.pi
                    print(
                        f"x: {x}, y: {y}, z: {z}, yaw: {math.degrees(yaw)} deg")
                    points.append((x, y, z, int(math.degrees(yaw))))
                total_grid[i][j][k] = points

    return total_grid

def get_trajectory_and_update_grid(total_grid: List[List[List]], x_vals, y_vals, z_vals):
    '''
    Generate a trajectory by moving through waypoints in the grid
    '''
    is_start_point_found = False
    while not is_start_point_found:
        start_x, start_y, start_z = random.randint(0, len(x_vals)-1), \
                                    random.randint(0, len(y_vals)-1), \
                                    random.randint(0, len(z_vals)-1)
        if len(total_grid[start_x][start_y][start_z]) > 0:
            is_start_point_found = True
    path = []
    queue = [(start_x, start_y, start_z)]
    while queue:
        x, y, z = queue.pop(0)
        if len(total_grid[x][y][z]) > 0:
            path.append(total_grid[x][y][z][0])
            del total_grid[x][y][z][0]

            all_candidates = list(
                product([x - 1, x, x + 1], [y - 1, y, y + 1], [z - 1, z, z + 1]))
            valid_candidates = []
            for i in all_candidates:
                x, y, z = i
                if x >= 0 and x < len(x_vals) and \
                   y >= 0 and y < len(y_vals) and \
                   z >= 0 and z < len(z_vals) and \
                   len(total_grid[x][y][z]) > 0:
                    valid_candidates.append((x, y, z))

            if valid_candidates:
                queue.append(random.sample(valid_candidates, k=1)[0])
    return path


class Drone:
    '''
    Drone object that navigates the drone in AirSim
    '''
    def __init__(self, x_min, x_max, y_min, y_max, alt_min, alt_max, speed=2) -> None:
        self.speed = speed
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.alt_min = alt_min
        self.alt_max = alt_max

    def connect_to_airsim(self) -> None:
        '''
        Connect to AirSim simulator
        '''
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def land(self) -> None:
        '''
        Land drone and disconnect API connection
        '''
        print("landing...")
        self.client.reset()
        print("disarming...")
        self.client.armDisarm(False)
        self.client.enableApiControl(False)
        print("done.")

    def start_grid_movement(self, 
                            start_path_flag: Event, 
                            finish_path_flag: Event, 
                            finish_flag: Event,
                            gen_all_paths: bool,
                            res_x=4, res_y=4, res_z=2, res_yaw=2) -> None:
        '''
        Generate a grid of waypoints based on spatial extent and resolution.
        This is used for generating training dataset
        '''
        self.x_vals = np.arange(self.x_min, self.x_max + 1, res_x)
        self.y_vals = np.arange(self.y_min, self.y_max + 1, res_y)
        self.z_vals = np.arange(self.alt_max, self.alt_min + 1, res_z)
        self.yaw_vals = np.arange(0, int(2*math.pi), res_yaw)
        print("x vals", self.x_vals)
        print("y vals", self.y_vals)
        print("z vals", self.z_vals)
        print("yaw vals", self.yaw_vals)

        total_grid = generate_grid(self.x_vals, self.y_vals, self.z_vals, self.yaw_vals) 
        paths = []

        if gen_all_paths:
            while has_items(flatten_nested_list(total_grid)):
                path = get_trajectory_and_update_grid(
                    total_grid, self.x_vals, self.y_vals, self.z_vals)
                paths.append(path)
        else:
            path = get_trajectory_and_update_grid(
                total_grid, self.x_vals, self.y_vals, self.z_vals)
            paths.append(path)

        print(paths)
        print("Path lengths")
        for path in paths:
            print(f"length of path = {len(path)}")

        for path in paths:
            if len(path) > 30:
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
                    self.client.moveToPositionAsync(float(x), float(y), float(
                        z), self.speed, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, yaw_mode=airsim.YawMode(False, float(yaw))).join()
                start_path_flag.clear()
                finish_path_flag.set()
                time.sleep(2)
                break

        time.sleep(2)
        finish_flag.set()
        self.land()

    def start_random_movement(self, 
                              start_path_flag: Event, 
                              finish_path_flag: Event, 
                              finish_flag: Event) -> None:
        '''
        Generate random waypoints and move the drones through the waypoints. 
        This is used to generate test dataset.
        '''
        def get_random_x_y_alt():
            return random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max), random.uniform(self.alt_min, self.alt_max)

        random_x_y_alt = [get_random_x_y_alt() for i in range(15)]
        print(random_x_y_alt)
        path = [airsim.Vector3r(*point) for point in random_x_y_alt]
        self.client.simSetVehiclePose(airsim.Pose(
            path[0], airsim.to_quaternion(0, 0, 0)), True)
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

    def start_debug_movement(self, 
                             start_path_flag: Event, 
                             finish_path_flag: Event, 
                             finish_flag: Event, 
                             predefined_path: List[Tuple[int, int, int]]) -> None:
        '''
        Start moving to a predefined path for debugging purposes
        '''
        predefined_path = [airsim.Vector3r(*xyz) for xyz in predefined_path]
        self.client.simSetVehiclePose(airsim.Pose(
            predefined_path[0], airsim.to_quaternion(0, 0, math.radians(270))), True)
        time.sleep(5)
        finish_path_flag.clear()
        start_path_flag.set()
        self.client.moveOnPathAsync(predefined_path, self.speed, 120,
                                    airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(False, 270)).join()
        start_path_flag.clear()
        finish_path_flag.set()

        time.sleep(2)
        finish_flag.set()
        self.land()