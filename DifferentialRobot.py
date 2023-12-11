from abc import ABC, abstractmethod
import math
import numpy as np
from controller import Robot, Supervisor

# from icecream import ic
from scipy.spatial.transform import Rotation as R


class DifferentialRobot(ABC):
    def __init__(self):
        self.me = Supervisor()
        self.node = self.me.getFromDef("ROBOT")
        self.timestep = int(self.me.getBasicTimeStep()) * 1
        self.max_wheel_linear_speed = 1  # m/s
        self.max_rotate_speed = 0.5
        self.v = {"vl": 0.0, "vr": 0.0}
        self.wheels = {"left": None, "right": None}
        self.steps = 0
        self.position = None
        self.rotationMatrix = None
        self.theta = 0.0
        self.initMotors()
        self.initSensors()

    def initMotors(self):
        self.wheels["left"] = self.me.getDevice("left wheel")
        self.wheels["left"].setPosition(float("+inf"))
        self.wheels["right"] = self.me.getDevice("right wheel")
        self.wheels["right"].setPosition(float("+inf"))
        self.v = {"vl": 0.0, "vr": 0.0}
        self.set_wheel_speeds()

    def set_wheel_speeds(self):
        self.wheels["left"].setVelocity(self.v["vl"])
        self.wheels["right"].setVelocity(self.v["vr"])
        # print(f"Speeds: vLeft: {self.v['vl']:2f}[m/s], vRight: {self.v['vr']:2f}[m/s]")

    def initSensors(self):
        self.lidar = self.me.getDevice("lidar")
        self.lidar.enable(self.timestep)
        self.lidar.enablePointCloud()
        self.lidarValues = []
        self.lidarDistances = []

    def update_position(self):
        self.position = self.node.getPosition().copy()
        self.rotationMatrix = self.node.getOrientation().copy()

        θz = math.atan2(self.rotationMatrix[3], self.rotationMatrix[0])

        self.theta = np.arccos(
            (
                self.rotationMatrix[0]
                + self.rotationMatrix[4]
                + self.rotationMatrix[8]
                - 1
            )
            / 2
        ) * (θz / abs(θz))

        if self.theta < 0:
            self.theta += 2 * math.pi

        print(
            f"Position: x: {self.position[0]:2f}[m], y: {self.position[1]:2f}[m], θ: {self.theta:2f}[rad]"
        )

    def update_sensors(self):
        self.lidarDistances = self.lidar.getRangeImage().copy()
        points = self.lidar.getPointCloud()  # .copy()
        points = np.vstack([[obj.x, obj.y, 0] for obj in points if obj.z == 0])
        r = R.from_matrix(np.array(self.rotationMatrix).reshape(3, 3))
        self.lidarValues = r.apply(points) + self.position

    def move_forward(self, speed):
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

    def rotate_counterclockwise(self, speed):
        self.v["vl"] = -speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] > -self.max_rotate_speed
            else -self.max_rotate_speed
        )
        self.v["vr"] = speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] < self.max_rotate_speed
            else self.max_rotate_speed
        )

    def rotate_clockwise(self, speed):
        self.v["vl"] = speed
        self.v["vl"] = (
            self.v["vl"]
            if self.v["vl"] < self.max_rotate_speed
            else self.max_rotate_speed
        )
        self.v["vr"] = -speed
        self.v["vr"] = (
            self.v["vr"]
            if self.v["vr"] > -self.max_rotate_speed
            else -self.max_rotate_speed
        )

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
            self.update()
            self.move()
            self.steps += 1
