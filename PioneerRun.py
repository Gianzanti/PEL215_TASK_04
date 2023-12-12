from itertools import cycle, islice
import math
import random

from matplotlib import animation, pyplot as plt
from DifferentialRobot import DifferentialRobot
import numpy as np

from icecream import ic
from GridMap import GridMap


class PioneerRun(DifferentialRobot):
    def __init__(self):
        super().__init__()
        random.seed(10)
        self.state = "find_next_target"

        self.target = {
            "ix": None,
            "iy": None,
            "px": None,
            "py": None,
            "θ": None,
            "avoiding": 0,
            "pre": False,
        }

        mapResolution = 0.25
        mapSize = (10, 10)
        mapWidth = int(mapSize[0] / mapResolution)
        mapHeight = int(mapSize[1] / mapResolution)
        self.map = GridMap(resolution=mapResolution, width=mapWidth, height=mapHeight)

        minimum = math.ceil(0.6 / mapResolution)

        self.predefined_targets = [
            {"x": minimum, "y": minimum},
            {"x": mapWidth - minimum, "y": mapHeight - minimum},
            {"x": minimum, "y": mapHeight - minimum},
            {"x": mapWidth - minimum, "y": minimum},
            {"x": minimum, "y": minimum},
            {"x": minimum, "y": mapHeight - minimum},
            {"x": mapWidth - minimum, "y": mapHeight - minimum},
            {"x": mapWidth - minimum, "y": minimum},
            {"x": int(mapWidth / 2), "y": minimum},
            {"x": int(mapWidth / 2), "y": int(mapHeight / 2)},
        ]
        self.lastRotateDirection = None

    def get_target_angle(self, target):
        delta_x = target["x"] - self.position[0]
        delta_y = target["y"] - self.position[1]

        theta = math.atan2(delta_y, delta_x)
        if theta < 0:
            theta += 2 * math.pi

        return theta

    def rotate(self):
        delta_theta = self.target["θ"] - self.theta

        if abs(delta_theta) < 0.01:
            self.lastRotateDirection = None
            self.stop()
            return True

        if delta_theta > math.pi:
            delta_theta -= 2 * math.pi
        elif delta_theta < -math.pi:
            delta_theta += 2 * math.pi

        if delta_theta > 0:
            if self.lastRotateDirection == "cw":
                self.lastRotateDirection = None
                self.stop()
                return True

            self.rotate_counterclockwise(self.max_wheel_linear_speed)
            self.lastRotateDirection = "ccw"
        else:
            if self.lastRotateDirection == "ccw":
                self.lastRotateDirection = None
                self.stop()
                return True

            self.rotate_clockwise(self.max_wheel_linear_speed)
            self.lastRotateDirection = "cw"

        return False

    def move(self):
        self.set_wheel_speeds()

    def odometry(self):
        if self.state != "stop":
            self.update_position()

    def cyclic_range(self, start, stop):
        return list(islice(cycle(range(stop)), start, start + stop))

    def find_target(self):
        cyclePos = self.map.get_xy_index_from_pos(self.position)
        cycleX = self.cyclic_range(cyclePos[0], self.map.width - 1)
        cycleY = self.cyclic_range(cyclePos[1], self.map.height - 1)

        # FIND ZERO VALUE TARGETS
        for x in cycleX:
            for y in cycleY:
                if (
                    self.map.grid[x][y] == 0
                    and (x != self.target["ix"] and y != self.target["iy"])
                    # and self.lastTargets[-1] != (x, y)
                ):
                    posX = x * self.map.resolution + self.map.origin_x
                    posY = y * self.map.resolution + self.map.origin_y
                    delta_x = posX - self.position[0]
                    delta_y = posY - self.position[1]
                    angle = math.atan2(delta_y, delta_x)
                    if angle < 0:
                        angle += 2 * math.pi

                    self.target = {
                        "ix": x,
                        "iy": y,
                        "px": posX,
                        "py": posY,
                        "θ": angle,
                        "avoiding": 0,
                        "value": self.map.grid[x][y],
                        "pre": False,
                    }
                    return True

        # PREDEFINED TARGETS
        if len(self.predefined_targets) > 0:
            predefined = self.predefined_targets[0]
            posX = predefined["x"] * self.map.resolution + self.map.origin_x
            posY = predefined["y"] * self.map.resolution + self.map.origin_y
            delta_x = posX - self.position[0]
            delta_y = posY - self.position[1]
            angle = math.atan2(delta_y, delta_x)
            if angle < 0:
                angle += 2 * math.pi

            self.target = {
                "ix": predefined["x"],
                "iy": predefined["y"],
                "px": posX,
                "py": posY,
                "θ": angle,
                "avoiding": 0,
                "value": self.map.grid[predefined["x"]][predefined["y"]],
                "pre": True,
            }
            return True
        else:
            return False

    def follow_target(self):
        delta_x = self.target["px"] - self.position[0]
        delta_y = self.target["py"] - self.position[1]

        # checks if the robot is close enough to the target
        if abs(delta_x) < 0.05 and abs(delta_y) < 0.05:
            ic("Close enough to target")
            if self.target["pre"]:
                if len(self.predefined_targets) > 0:
                    self.predefined_targets.pop(0)
            self.stop()
            self.state = "find_next_target"
            return True

        # avoid obstacles
        left_side = self.lidarDistances[150:180]
        right_side = self.lidarDistances[180:210]
        obstacle_left = min(left_side) <= 0.4
        obstacle_right = min(right_side) <= 0.4
        left_value = sum(
            [len(left_side) if x == float("inf") else x for x in left_side]
        ) / len(left_side)
        right_value = sum(
            [len(right_side) if x == float("inf") else x for x in right_side]
        ) / len(right_side)

        if obstacle_left or obstacle_right:
            if left_value < right_value:
                ic("avoiding obstacles at left")
                angle = self.theta - math.pi / 4
            else:
                ic("avoiding obstacles at right")
                angle = self.theta + math.pi / 4

            # Calculate new coordinates
            iPos = None
            while iPos == None:
                dist = (
                    2 if self.target["avoiding"] == 0 else self.target["avoiding"] + 1
                )
                new_x = self.position[0] + ((random.random() * dist) * math.cos(angle))
                new_y = self.position[1] + ((random.random() * dist) * math.sin(angle))
                iPos = self.map.get_xy_index_from_pos([new_x, new_y])

            delta_x = new_x - self.position[0]
            delta_y = new_y - self.position[1]
            angle = math.atan2(delta_y, delta_x)
            if angle < 0:
                angle += 2 * math.pi

            self.target = {
                "ix": iPos[0],
                "iy": iPos[1],
                "px": new_x,
                "py": new_y,
                "θ": angle,
                "avoiding": dist if dist <= 5 else 5,
                "value": self.map.grid[iPos[0]][iPos[1]],
                "pre": False,
            }

            # ic("avoiding obstacles")
            self.state = "rotate_to_target"
            return True

    def update(self):
        ic(self.state, self.target)
        self.update_sensors()
        self.map.set_occupancy_grid(
            self.lidarValues, [self.position[0], self.position[1]]
        )

        match self.state:
            case "find_next_target":
                foundTarget = self.find_target()
                if not foundTarget:
                    ic(self.steps)
                    self.state = "stop"
                    ic(self.map.grid)

                    np.savez_compressed(f"./data_grid.npz", grid=self.map.grid)

                    new_grid = np.rot90(self.map.grid, k=3)
                    new_grid = np.fliplr(new_grid)
                    plt.pcolor(
                        new_grid,
                        cmap="Blues",
                        edgecolor="tab:gray",
                        linewidths=1,
                    )
                    plt.title("Occupancy Grid")
                    plt.tight_layout()
                    plt.axis("equal")
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

            case "stop":
                self.stop()
