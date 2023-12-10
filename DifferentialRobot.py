from abc import ABC, abstractmethod
import math
import numpy as np
from controller import Robot, Supervisor
from icecream import ic

INF = float("+inf")


class DifferentialRobot(ABC):
    def __init__(self):
        self.me = Supervisor()

        robotDef = "ROBOT"
        self.node = self.me.getFromDef(robotDef)

        self.timestep = int(self.me.getBasicTimeStep()) * 1

        max_linear_velocity = 12.3  # rad/s
        # self.wheel_radius = 0.0975  # m
        # self.max_wheel_linear_speed = max_linear_velocity * self.wheel_radius  # m/s
        self.max_wheel_linear_speed = 1  # m/s
        # self.wheel_speed_increment = 0.5 * self.max_wheel_linear_speed
        self.mat_rotate_speed = 0.5
        self.l = {"x": 0.485, "y": 0.381, "z": 0.217}  # m
        self.robot_radius = max(self.l["x"], self.l["y"]) * 2  # m

        self.p = {"x": 0.0, "y": 0.0, "θ": 0.0}
        self.v = {"vl": 0.0, "vr": 0.0, "ω": 0.0}

        self.wheels = {"left": None, "right": None}
        self.steps = 0

        self.position = None
        self.rotation = None
        self.pose = None

        self.initMotors()
        self.initSensors()

    def initMotors(self):
        self.wheels["left"] = self.me.getDevice("left wheel")
        self.wheels["left"].setPosition(INF)
        self.wheels["right"] = self.me.getDevice("right wheel")
        self.wheels["right"].setPosition(INF)
        self.v = {"vl": 0.0, "vr": 0.0, "ω": 0.0}

        self.set_wheel_speeds()

    def set_wheel_speeds(self):
        self.wheels["left"].setVelocity(self.v["vl"])
        self.wheels["right"].setVelocity(self.v["vr"])

    def initSensors(self):
        self.lidar = self.me.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.lidarValues = []

    def base_move(self):
        self.set_wheel_speeds()
        print(
            f"Speeds: vLeft: {self.v['vl']:2f}[m/s], vRight: {self.v['vr']:2f}[m/s], ω: {self.v['ω']:2f}[rad/s]"
        )

    def update_position(self):
        self.pose = self.node.getPose().copy()
        # ic(self.pose)

        self.position = self.node.getPosition().copy()
        ic(self.position)

        self.p["x"] = self.position[0]
        self.p["y"] = self.position[1]
        self.rotation = self.node.getOrientation().copy()
        # ic(self.rotation)

        θz = math.atan2(self.rotation[3], self.rotation[0])

        self.p["θ"] = np.arccos(
            (self.rotation[0] + self.rotation[4] + self.rotation[8] - 1) / 2
        ) * (θz / abs(θz))

        if self.p["θ"] < 0:
            self.p["θ"] += 2 * math.pi

        print(
            f'Position: x: {self.p["x"]:2f}[m], y: {self.p["y"]:2f}[m], θ: {self.p["θ"]:2f}[rad]'
        )

    def move_forward(self, speed):
        self.v["ω"] = 0
        self.v["vl"] += speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] < self.max_wheel_linear_speed
            else self.max_wheel_linear_speed
        )
        self.v["vr"] += speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] < self.max_wheel_linear_speed
            else self.max_wheel_linear_speed
        )

    def move_backward(self, speed):
        self.v["ω"] = 0
        self.v["vl"] -= speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] > -self.max_wheel_linear_speed
            else -self.max_wheel_linear_speed
        )
        self.v["vr"] -= speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] > -self.max_wheel_linear_speed
            else -self.max_wheel_linear_speed
        )

    def stop(self):
        self.v["vl"] = 0
        self.v["vr"] = 0
        self.v["ω"] = 0

    def rotate_counterclockwise(self, speed):
        self.v["vl"] = -speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] > -self.mat_rotate_speed
            else -self.mat_rotate_speed
        )
        self.v["vr"] = speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] < self.mat_rotate_speed
            else self.mat_rotate_speed
        )
        self.v["ω"] = (self.robot_radius / self.l["x"]) * (self.v["vr"] - self.v["vl"])

    def rotate_clockwise(self, speed):
        self.v["vl"] = speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] < self.mat_rotate_speed
            else self.mat_rotate_speed
        )
        self.v["vr"] = -speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] > -self.mat_rotate_speed
            else -self.mat_rotate_speed
        )
        self.v["ω"] = (self.robot_radius / self.l["x"]) * (self.v["vr"] - self.v["vl"])

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def move(self):
        pass

    @abstractmethod
    def odometry(self):
        pass

    def run(self):
        while self.me.step(self.timestep) != -1:
            self.odometry()
            self.lidarValues = self.lidar.getPointCloud().copy()
            # for idx, t in enumerate(tt):
            #     if t.z == 0:
            #         ic(idx, t.x, t.y, t.z)

            # self.lidarValues = self.lidar.getRangeImage().copy()
            self.update()
            self.move()
            self.steps += 1
