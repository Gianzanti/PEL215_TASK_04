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
        random.seed(10)
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
            "pre": False,
        }

        self.map = GridMap()
        self.lastRotateDirection = None
        self.lastTargets = [None, None]
        self.laser_beams = None
        self.predefined_targets = [
            {"x": 2, "y": 2},
            {"x": 2, "y": 18},
            {"x": 18, "y": 18},
            {"x": 18, "y": 2},
            {"x": 10, "y": 10},
        ]

        # steps = 1000
        # self.data_predictions = np.zeros((steps,), dtype="f,f")
        # self.data_measurements = np.zeros((steps,), dtype="f,f")
        # self.data_corrections = np.zeros((steps,), dtype="f,f")

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
        r = R.from_matrix(np.array(self.rotation).reshape(3, 3))
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
        xx = self.map.calc_xy_index_from_pos(
            self.p["x"], self.map.origin_x, self.map.width
        )
        yy = self.map.calc_xy_index_from_pos(
            self.p["y"], self.map.origin_y, self.map.height
        )

        cycleX = self.cyclic_range(xx, self.map.width - 1)
        cycleY = self.cyclic_range(yy, self.map.height - 1)

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
                        "pre": False,
                    }
                    ic("Zero Target found **********************************")
                    ic(self.target)
                    self.lastTargets.append((x, y))
                    self.lastTargets.pop()
                    ic(self.lastTargets)
                    return True

        # PREDEFINED TARGETS
        if len(self.predefined_targets) > 0:
            predefined = self.predefined_targets[0]
            posX = predefined["x"] * self.map.resolution + self.map.origin_x
            posY = predefined["y"] * self.map.resolution + self.map.origin_y
            delta_x = posX - self.p["x"]
            delta_y = posY - self.p["y"]
            angle = math.atan2(delta_y, delta_x)

            self.target = {
                "ix": predefined["x"],
                "iy": predefined["y"],
                "px": posX,
                "py": posY,
                "θ": angle,
                "avoiding": False,
                "value": self.map.grid[predefined["x"]][predefined["y"]],
                "pre": True,
            }
            ic("Predefined Target found **********************************")
            ic(self.target)
            self.lastTargets.append((x, y))
            self.lastTargets.pop()
            ic(self.lastTargets)
            return True
        else:
            return False

        # target = {"x": 0, "y": 0, "value": float("inf")}
        # cycleX = self.cyclic_range(
        #     int(random.random() * self.map.width), self.map.width - 1
        # )
        # cycleY = self.cyclic_range(
        #     int(random.random() * self.map.height), self.map.height - 1
        # )
        # for x in cycleX:
        #     for y in cycleY:
        #         value = self.map.grid[x][y]
        #         ic(value)

        #         more_then_upperLimit = value > 0.8 * self.map.thresholdOccupied  # 2400
        #         less_then_lowerLimit = value < 0.5 * self.map.thresholdFree  # - 2400

        #         if (
        #             (not less_then_lowerLimit and not more_then_upperLimit)
        #             and self.lastTargets[-1] != (x, y)
        #             and x != self.target["ix"]
        #             and y != self.target["iy"]
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
            if self.target["pre"]:
                self.predefined_targets.pop(0)
            self.stop()
            self.state = "find_next_target"
            return True

        # avoid obstacles
        left_side = self.lidarDistances[160:180]
        right_side = self.lidarDistances[180:200]
        obstacle_left = min(left_side) < 0.65
        obstacle_right = min(right_side) < 0.65
        left_value = sum([5 if x == float("inf") else x for x in left_side]) / 10
        right_value = sum([5 if x == float("inf") else x for x in right_side]) / 10

        if obstacle_left or obstacle_right:
            if left_value < right_value:
                ic("avoiding obstacles at left")
                angle = self.target["θ"] - math.pi / 6
            else:
                ic("avoiding obstacles at right")
                angle = self.target["θ"] + math.pi / 6

            # Calculate new coordinates
            new_x = self.p["x"] + ((random.random() * 3) - 2) * math.cos(angle)
            new_x = new_x if (new_x <= 9.5) else 9.5
            new_x = new_x if (new_x >= 1) else 1
            new_y = self.p["y"] + ((random.random() * 3) - 2) * math.sin(angle)
            new_y = new_y if (new_y <= 9.5) else 9.5
            new_y = new_y if (new_y >= 1) else 1

            iPosX = self.map.calc_xy_index_from_pos(
                new_x, self.map.origin_x, self.map.width
            )
            iPosY = self.map.calc_xy_index_from_pos(
                new_y, self.map.origin_y, self.map.height
            )

            delta_x = new_x - self.p["x"]
            delta_y = new_y - self.p["y"]
            angle = math.atan2(delta_y, delta_x)

            self.target = {
                "ix": iPosX,
                "iy": iPosY,
                "px": new_x,
                "py": new_y,
                "θ": angle,
                "avoiding": True,
                "value": None,
                "pre": False,
            }

            # ic("avoiding obstacles")
            self.state = "rotate_to_target"
            return True

        # checks if target is already checked
        target_value = self.map.grid[self.target["ix"]][self.target["iy"]]
        # ic(self.target, target_value)
        if (
            target_value != self.target["value"]
            and not self.target["avoiding"]
            and not self.target["pre"]
        ):
            ic("**************************** Target already checked")
            self.stop()
            self.state = "find_next_target"
            return True

    def update(self):
        ic(self.state, self.target)

        self.laser_beams = self.read_lidar()
        self.map.set_occupancy_grid(self.laser_beams, [self.p["x"], self.p["y"]])

        np.savez_compressed(f"./data_history.npz", grid=self.map.grid)

        # ic(self.map.grid)
        # self.historyGrid.append(self.map.grid.copy())
        # self.historyBeams.append(self.laser_beams.copy())

        # np.savez_compressed(
        #     f"./data_history.npz",
        #     timeStep=self.timestep,
        #     beams=self.historyBeams,
        #     grid=self.historyGrid,
        # )

        # if self.state != "rotate_to_target":
        #     plt.figure(1, figsize=(10, 4))
        #     plt.subplot(122)
        #     # plt.imshow(new_grid, cmap="bone_r")
        #     plt.pcolor(self.map.grid, cmap="Blues")  # , vmin=0.0, vmax=1.0)
        #     plt.axis("equal")

        #     # return heat_map

        #     plt.subplot(121)

        #     plt.plot(
        #         [
        #             self.laser_beams[:, 0],
        #             np.full(np.size(self.laser_beams[:, 0]), self.p["x"]),
        #         ],
        #         [
        #             self.laser_beams[:, 1],
        #             np.full(np.size(self.laser_beams[:, 1]), self.p["y"]),
        #         ],
        #         "ro-",
        #     )

        #     plt.plot(self.p["x"], self.p["y"], "ob")
        #     plt.gca().set_aspect("equal", "box")
        #     plt.show()

        match self.state:
            case "find_next_target":
                # # check current position index
                # iPosX = int(
                #     round((self.p["x"] - self.map.origin_x) / self.map.resolution)
                # )
                # iPosY = int(
                #     round((self.p["y"] - self.map.origin_y) / self.map.resolution)
                # )

                # # ic(iPosX, iPosY)

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

                    plt.figure(1, figsize=(10, 4))
                    plt.subplot(122)
                    # plt.imshow(new_grid, cmap="bone_r")
                    plt.pcolor(self.map.grid, cmap="Blues")  # , vmin=0.0, vmax=1.0)
                    plt.axis("equal")

                    # return heat_map

                    plt.subplot(121)

                    plt.plot(
                        [
                            self.laser_beams[:, 0],
                            np.full(np.size(self.laser_beams[:, 0]), self.p["x"]),
                        ],
                        [
                            self.laser_beams[:, 1],
                            np.full(np.size(self.laser_beams[:, 1]), self.p["y"]),
                        ],
                        "ro-",
                    )

                    plt.plot(self.p["x"], self.p["y"], "ob")
                    plt.gca().set_aspect("equal", "box")
                    plt.show()
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
