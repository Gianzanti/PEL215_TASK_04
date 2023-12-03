import math

from matplotlib import pyplot as plt
from DifferentialRobot import DifferentialRobot
import numpy as np
from icecream import ic
from Map import Map

class PioneerRun(DifferentialRobot):
    def __init__(self):
        super().__init__()
        # self.state = "find-next-door"
        # self.state = "stop"
        self.state = "find_next_target"

        self.map = Map()
        # self.map.show()

        steps = 1000
        self.data_predictions = np.zeros((steps,), dtype='f,f')
        self.data_measurements = np.zeros((steps,), dtype='f,f')
        self.data_corrections = np.zeros((steps,), dtype='f,f')

    def get_target_angle(self, target):
        delta_x = target["x"] - self.p["x"]
        delta_y = target["y"] - self.p["y"]
        return math.atan2(delta_y, delta_x)
    
    def rotate(self, angle: float):
        delta_theta = angle - self.p["θ"]
        
        if abs(delta_theta) < 0.01:
            self.stop()
            return True

        if delta_theta < 0:
            self.rotate_counterclockwise(self.max_wheel_linear_speed)
        
        elif delta_theta > 0:
            self.rotate_clockwise(self.max_wheel_linear_speed)

        return False

    def move(self):
        self.base_move()

    def odometry(self):
        if (self.state != "stop"):
            self.update_position()
    
    def read_lidar(self):
        # lidar_values = self.lidar.getRangeImage()
        angles = []
        distances = []

        for angle, measure in enumerate(self.lidarValues):
            if (measure == float('inf')):
                continue
            angles.append(math.radians(angle)+ self.p["θ"]+ math.pi)
            distances.append(float(measure))

        angles = np.array(angles)
        distances = np.array(distances)
        
        ox = np.sin(angles) * distances
        oy = np.cos(angles) * distances
        
        return ox, oy

    def find_target(self):
        self.target = {'x': None, 'y': None}

        for x in range(0, self.map.width):
            for y in range(0, self.map.height):
                if (self.map.grid[x][y] == 0):
                    self.target = {'x': x, 'y': y}
                    ic('Target found *************************************************')
                    ic(self.target)
                    return True
        
        return False

    def follow_target(self):
        delta_x = self.target["x"] - self.p["x"]
        delta_y = self.target["y"] - self.p["y"]

        # checks if the robot is close enough to the target
        if (abs(delta_x) < 0.01 and abs(delta_y) < 0.01):
            ic('Close enough to target')
            self.stop()
            self.state = 'find_next_target'
            return True

        # checks if target is already checked
        target_value = self.map.grid[self.target["x"]][self.target["y"]]
        ic(self.target, target_value)
        if (target_value != 0):
            ic('**************************** Target already checked')            
            self.stop()
            self.state = 'find_next_target'
            return True


        # avoid obstacles
        # 171 a 191
        for i in range(171, 191):
            if (self.lidarValues[i] < 0.5):
                ic('avoiding obstacles')
                self.state = "avoid_obstacles"
                return True


    def update(self):
        ic(self.state)
        # ic(self.lidarValues)

        ox, oy = self.read_lidar()
        self.map.set_occupancy_grid(ox, oy, [self.p['x'], self.p['y']])
        # ic(self.map.grid)
            #     np.savez_compressed(
            #         f"./data_kalman.npz",
            #         timeStep=self.timestep,
            #         predictions=self.data_predictions,
            #         measurements=self.data_measurements,
            #         corrections=self.data_corrections,
            #     )

        # # xy_resolution = 0.1  # x-y grid resolution
        # # occupancy_map, min_x, max_x, min_y, max_y, xy_resolution = generate_ray_casting_grid_map(ox, oy, xy_resolution, True)
        # xy_res = np.array(self.map.grid).shape
        # plt.figure(1, figsize=(10, 4))
        # plt.subplot(122)
        # # plt.imshow(self.map.grid, cmap="PiYG_r")
        # plt.imshow(self.map.grid, cmap="bone_r")
        # # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
        # plt.clim(-10, 10)
        # plt.gca().set_xticks(np.arange(-.5, xy_res[1], 1), minor=True)
        # plt.gca().set_yticks(np.arange(-.5, xy_res[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        # plt.colorbar()
        # plt.subplot(121)
        # plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
        # plt.axis("equal")
        # plt.plot(0.0, 0.0, "ob")
        # plt.gca().set_aspect("equal", "box")
        # bottom, top = plt.ylim()  # return the current y-lim
        # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # plt.grid(True)
        # plt.show()


        match self.state:
            case "find_next_target":
                foundTarget = self.find_target()
                if (not foundTarget):
                    self.state = "stop"
                    return

                ic(self.target)
                self.state = "rotate_to_target"

            case "move_to_target":
                if (self.follow_target()):
                    # self.state = "rotate_to_target"
                    return
                
                self.move_forward(self.max_wheel_linear_speed)


            case "rotate_to_target":
                target_angle = self.get_target_angle(self.target)

                if (self.rotate(target_angle)):
                    self.state = "move_to_target"

            case "avoid_obstacles":
                turn45 = self.p['θ'] + math.pi/4
                ic(turn45)
                if (self.rotate(turn45)):
                    self.state = "move_to_target"

            # case "graph":
            #     self.stop()
            #     self.state = "stop"
            #     np.savez_compressed(
            #         f"./data_kalman.npz",
            #         timeStep=self.timestep,
            #         predictions=self.data_predictions,
            #         measurements=self.data_measurements,
            #         corrections=self.data_corrections,
            #     )

            case "stop":
                self.stop()
                
