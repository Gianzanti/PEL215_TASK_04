from itertools import cycle, islice
import math
import random

from matplotlib import pyplot as plt
from DifferentialRobot import DifferentialRobot
import numpy as np
from scipy.spatial.transform import Rotation as R
from icecream import ic
from GridMap import GridMap


class PioneerRun(DifferentialRobot):
    def __init__(self):
        super().__init__()
        # self.state = "find-next-door"
        # self.state = "stop"
        self.state = "find_next_target"
        self.target = {
            "ix": None,
            "iy": None,
            "px": None,
            "py": None,
            "θ": None,
            "avoiding": False,
        }

        self.map = GridMap()
        self.lastRotateDirection = None
        self.lastTargets = [None, None]

        # self.map.show()

        # plt.ion()  # enable real-time plotting
        # plt.figure(1, figsize=(10, 4))
        # xy_res = np.array(self.map.grid).shape
        # plt.subplot(122)
        # plt.imshow(self.map.grid, cmap="bone_r")
        # plt.clim(-10, 10)
        # plt.gca().set_xticks(np.arange(-0.5, xy_res[1], 1), minor=True)
        # plt.gca().set_yticks(np.arange(-0.5, xy_res[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        # plt.colorbar()

        steps = 1000
        self.data_predictions = np.zeros((steps,), dtype="f,f")
        self.data_measurements = np.zeros((steps,), dtype="f,f")
        self.data_corrections = np.zeros((steps,), dtype="f,f")

    def get_target_angle(self, target):
        delta_x = target["x"] - self.p["x"]
        delta_y = target["y"] - self.p["y"]
        return math.atan2(delta_y, delta_x)

    def rotate(self):
        delta_theta = self.target["θ"] - self.p["θ"]
        # ic(delta_theta)

        if abs(delta_theta) < 0.01:
            self.lastRotateDirection = None
            self.stop()
            return True

        if delta_theta > math.pi:
            delta_theta -= 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta += 2 * math.pi
        # ic(delta_theta)

        if delta_theta > 0:
            if self.lastRotateDirection == "cw":
                self.lastRotateDirection = None
                self.stop()
                return True

            # ic("rotate counterclockwise")
            self.rotate_counterclockwise(self.max_wheel_linear_speed)
            self.lastRotateDirection = "ccw"
        else:
            if self.lastRotateDirection == "ccw":
                self.lastRotateDirection = None
                self.stop()
                return True

            # ic("rotate clockwise")
            self.rotate_clockwise(self.max_wheel_linear_speed)
            self.lastRotateDirection = "cw"

        # ox, oy = self.read_lidar()
        # self.map.set_occupancy_grid(ox, oy, [self.p["x"], self.p["y"]])
        return False

    def move(self):
        self.base_move()

    def odometry(self):
        # if self.state != "stop":
        self.update_position()

    def read_lidar(self):
        # a = np.array(self.rotation).reshape(3, 3)
        r = R.from_matrix(np.array(self.rotation).reshape(3, 3))
        # matrix = r.as_matrix()
        # ic(matrix)
        # ic(np.linalg.det(matrix))

        # Create a single NumPy array from the list of objects
        combined_array = np.vstack(
            [[obj.x, obj.y, obj.z] for obj in self.lidarValues if obj.z == 0]
        )

        tt = r.apply(combined_array)
        tt = tt + self.position
        return tt

    def cyclic_range(self, start, stop):
        return list(islice(cycle(range(stop)), start, start + stop))

    def find_target(self):
        xx = int(round((self.p["x"] - self.map.origin_x) / self.map.resolution))
        yy = int(round((self.p["y"] - self.map.origin_y) / self.map.resolution))

        cycleX = self.cyclic_range(xx, self.map.width - 1)
        # ic(cycleX)
        cycleY = self.cyclic_range(yy, self.map.height - 1)
        # ic(cycleY)

        for x in cycleX:
            for y in cycleY:
                if (
                    self.map.grid[x][y] == 0
                    and x != self.target["ix"]
                    and y != self.target["iy"]
                    and self.lastTargets[-1] != (x, y)
                ):
                    posX = x * self.map.resolution + self.map.origin_x
                    posY = y * self.map.resolution + self.map.origin_y
                    delta_x = posX - self.p["x"]
                    delta_y = posY - self.p["y"]
                    angle = math.atan2(delta_y, delta_x)

                    self.target = {
                        "ix": x,
                        "iy": y,
                        "px": posX,
                        "py": posY,
                        "θ": angle,
                        "avoiding": False,
                        "value": self.map.grid[x][y],
                    }
                    ic("Zero Target found **********************************")
                    ic(self.target)
                    self.lastTargets.append((x, y))
                    self.lastTargets.pop()
                    ic(self.lastTargets)
                    return True

        # target = {"x": 0, "y": 0, "value": float("inf")}

        # for x in range(1, self.map.width - 1):
        #     for y in range(1, self.map.height - 1):
        #         value = self.map.grid[x][y]
        #         ic(value)

        #         lowerLimit = value > 0.7 * self.map.thresholdOccupied
        #         ic(0.6 * self.map.thresholdOccupied)
        #         ic(lowerLimit)

        #         upperLimit = value < 0.7 * self.map.thresholdFree
        #         ic(0.6 * self.map.thresholdFree)
        #         ic(upperLimit)

        #         if (
        #             not lowerLimit
        #             and not upperLimit
        #             and value < target["value"]
        #             and x != self.target["ix"]
        #             and y != self.target["iy"]
        #             and self.lastTargets[-1] != (x, y)
        #         ):
        #             target = {"x": x, "y": y, "value": value}

        # if target["value"] != float("inf"):
        #     ic("Undefined Target found ********************************")
        #     ic(target)

        #     posX = target["x"] * self.map.resolution + self.map.origin_x
        #     posY = target["y"] * self.map.resolution + self.map.origin_y
        #     delta_x = posX - self.p["x"]
        #     delta_y = posY - self.p["y"]
        #     angle = math.atan2(delta_y, delta_x)

        #     self.target = {
        #         "ix": target["x"],
        #         "iy": target["y"],
        #         "px": posX,
        #         "py": posY,
        #         "θ": angle,
        #         "avoiding": False,
        #         "value": value,
        #     }

        #     self.lastTargets.append((target["x"], target["y"]))
        #     self.lastTargets.pop()
        #     ic(self.lastTargets)
        #     return True

        return False

    def follow_target(self):
        delta_x = self.target["px"] - self.p["x"]
        delta_y = self.target["py"] - self.p["y"]

        # checks if the robot is close enough to the target
        if abs(delta_x) < 0.05 and abs(delta_y) < 0.05:
            ic("Close enough to target")
            self.stop()
            self.state = "find_next_target"
            return True

        # # avoid obstacles
        # left_side = self.lidarValues[340:360]
        # right_side = self.lidarValues[0:20]
        # obstacle_left = min(left_side) < 0.65
        # obstacle_right = min(right_side) < 0.65
        # left_value = sum([5 if x == float("inf") else x for x in left_side]) / 10
        # right_value = sum([5 if x == float("inf") else x for x in right_side]) / 10

        # if obstacle_left or obstacle_right:
        #     if left_value < right_value:
        #         ic("avoiding obstacles at left")
        #         angle = self.target["θ"] - math.pi / 4
        #     else:
        #         ic("avoiding obstacles at right")
        #         angle = self.target["θ"] + math.pi / 4

        #     # Calculate new coordinates
        #     new_x = self.p["x"] + ((random.random() * 4) - 2) * math.cos(angle)
        #     new_x = new_x if (new_x <= 9.5) else 9.5
        #     new_x = new_x if (new_x >= 1) else 1
        #     new_y = self.p["y"] + ((random.random() * 4) - 2) * math.sin(angle)
        #     new_y = new_y if (new_y <= 9.5) else 9.5
        #     new_y = new_y if (new_y >= 1) else 1

        #     iPosX = int(round((new_x - self.map.origin_x) / self.map.resolution))
        #     iPosY = int(round((new_y - self.map.origin_y) / self.map.resolution))

        #     delta_x = new_x - self.p["x"]
        #     delta_y = new_y - self.p["y"]
        #     angle = math.atan2(delta_y, delta_x)

        #     self.target = {
        #         "ix": iPosX,
        #         "iy": iPosY,
        #         "px": new_x,
        #         "py": new_y,
        #         "θ": angle,
        #         "avoiding": True,
        #         "value": None,
        #     }

        #     # ic("avoiding obstacles")
        #     self.state = "rotate_to_target"
        #     return True

        # checks if target is already checked
        target_value = self.map.grid[self.target["ix"]][self.target["iy"]]
        # ic(self.target, target_value)
        if target_value != self.target["value"] and not self.target["avoiding"]:
            ic("**************************** Target already checked")
            self.stop()
            self.state = "find_next_target"
            return True

    def update(self):
        ic(self.state, self.target)

        laser_beams = self.read_lidar()
        self.map.set_occupancy_grid(laser_beams, [self.p["x"], self.p["y"]])

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
        # # xy_res = np.array(self.map.grid).shape
        # # plt.ion()  # enable real-time plotting
        # # plt.figure(1, figsize=(10, 4))
        # plt.subplot(122)
        # # plt.imshow(self.map.grid, cmap="PiYG_r")
        # plt.imshow(self.map.grid, cmap="bone_r")
        # # cmap = "binary" "PiYG_r" "PiYG_r" "bone" "bone_r" "RdYlGn_r"
        # plt.clim(np.max(self.map.grid), np.min(self.map.grid))
        # # plt.gca().set_xticks(np.arange(-0.5, xy_res[1], 1), minor=True)
        # # plt.gca().set_yticks(np.arange(-0.5, xy_res[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        # plt.colorbar()
        # # plt.subplot(121)
        # # plt.plot([oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-")
        # # plt.axis("equal")
        # # plt.plot(0.0, 0.0, "ob")
        # # plt.gca().set_aspect("equal", "box")
        # # bottom, top = plt.ylim()  # return the current y-lim
        # # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # # plt.grid(True)
        # # plt.ioff()
        # # plt.clf()
        # # plt.show()
        # plt.pause(1 / 10000)

        # Create a copy of the array
        # new_grid = self.map.grid.copy()

        # # Zero out the elements of the copied array
        # new_grid.fill(0)

        # for x in range(0, self.map.width):
        #     for y in range(0, self.map.height):
        #         if self.map.grid[x][y] > 0:
        #             new_grid[x][y] = 1
        #         elif self.map.grid[x][y] < 0:
        #             new_grid[x][y] = -1
        #         else:
        #             new_grid[x][y] = 0

        # xy_res = np.array(new_grid).shape
        plt.figure(1, figsize=(10, 4))
        plt.subplot(122)
        # plt.imshow(new_grid, cmap="bone_r")

        plt.pcolor(self.map.grid, cmap="Blues")  # , vmin=0.0, vmax=1.0)
        plt.axis("equal")

        # return heat_map

        # plt.clim(-1, 1)
        # plt.gca().set_xticks(np.arange(-0.5, xy_res[1], 1), minor=True)
        # plt.gca().set_yticks(np.arange(-0.5, xy_res[0], 1), minor=True)
        # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
        # plt.colorbar()
        plt.subplot(121)

        plt.plot(
            [laser_beams[:, 0], np.full(np.size(laser_beams[:, 0]), self.p["x"])],
            [laser_beams[:, 1], np.full(np.size(laser_beams[:, 1]), self.p["y"])],
            "ro-",
        )

        # plt.axis("equal")
        plt.plot(self.p["x"], self.p["y"], "ob")
        plt.gca().set_aspect("equal", "box")
        # bottom, top = plt.ylim()  # return the current y-lim
        # plt.ylim((top, bottom))  # rescale y axis, to match the grid orientation
        # plt.grid(True)
        plt.show()

        match self.state:
            case "find_next_target":
                # check current position index
                iPosX = int(
                    round((self.p["x"] - self.map.origin_x) / self.map.resolution)
                )
                iPosY = int(
                    round((self.p["y"] - self.map.origin_y) / self.map.resolution)
                )

                # ic(iPosX, iPosY)

                foundTarget = self.find_target()
                if not foundTarget:
                    self.state = "stop"
                    ic(self.map.grid)

                    np.savez_compressed(f"./data_grid.npz", grid=self.map.grid)

                    # # Create a copy of the array
                    # new_grid = self.map.grid.copy()

                    # # Zero out the elements of the copied array
                    # new_grid.fill(0)

                    # for x in range(0, self.map.width):
                    #     for y in range(0, self.map.height):
                    #         if self.map.grid[x][y] > 0:
                    #             new_grid[x][y] = 1
                    #         elif self.map.grid[x][y] < 0:
                    #             new_grid[x][y] = -1
                    #         else:
                    #             new_grid[x][y] = 0

                    # xy_res = np.array(new_grid).shape
                    # plt.figure(1, figsize=(10, 4))
                    # plt.subplot(122)
                    # plt.imshow(new_grid, cmap="bone_r")
                    # plt.clim(-1, 1)
                    # plt.gca().set_xticks(np.arange(-0.5, xy_res[1], 1), minor=True)
                    # plt.gca().set_yticks(np.arange(-0.5, xy_res[0], 1), minor=True)
                    # plt.grid(True, which="minor", color="w", linewidth=0.6, alpha=0.5)
                    # plt.colorbar()
                    # plt.subplot(121)
                    # plt.plot(
                    #     [oy, np.zeros(np.size(oy))], [ox, np.zeros(np.size(oy))], "ro-"
                    # )
                    # plt.axis("equal")
                    # plt.plot(0.0, 0.0, "ob")
                    # plt.gca().set_aspect("equal", "box")
                    # bottom, top = plt.ylim()  # return the current y-lim
                    # plt.ylim(
                    #     (top, bottom)
                    # )  # rescale y axis, to match the grid orientation
                    # plt.grid(True)
                    # plt.show()
                    return

                self.state = "rotate_to_target"

            case "move_to_target":
                if self.follow_target():
                    return

                self.move_forward(self.max_wheel_linear_speed)

            case "rotate_to_target":
                if self.rotate():
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
