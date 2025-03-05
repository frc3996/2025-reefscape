import math
from concurrent import futures
from typing import List

import wpilib
from magicbot import tunable
from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d

from common.limelight_helpers import LimelightHelpers, PoseEstimate
from components.gyro import Gyro
from components.limelight import LimeLightVision
from components.swervedrive import SwerveDrive


class VisionSystem:

    # cameras: list[LimeLightVision]
    limelightFront: LimeLightVision
    limelightBack: LimeLightVision
    gyro: Gyro

    __threaded: bool = False
    __executor: futures.ThreadPoolExecutor | None
    drivetrain: SwerveDrive

    def __init__(self):
        self.__executor = None
        self.__threaded = False
        if self.__threaded:
            self.__executor = futures.ThreadPoolExecutor()

    def setup(self):
        self.cameras = [self.limelightFront, self.limelightBack]

    def miseAjour(self):
        if self.gyro.yawSpeed() > 720:
            print("VisionSystem : skipping, yaw rot speed too high")
            return

        # TODO Est-ce qu'on devrait utiliser l'estimé du drivetrain ou le gyro?
        # robotRotation = self.drivetrain.getPose().rotation()
        # if self.gyro.ok():
        robotRotation = self.gyro.getRotation2d()
        # robotRotation = self.gyro.getRotation2d()

        if not self.__threaded:
            for cam in self.cameras:
                ret = cam.getVisionMesurement(robotRotation)
                if ret is not None:
                    pose: Pose2d
                    timestamp: float
                    stddevs: tuple[float, float, float]
                    pose, timestamp, stddevs = ret
                    print(pose, timestamp, stddevs)
                    self.drivetrain.addVisionPoseEstimate(pose, timestamp, stddevs)
        else:
            assert self.__executor is not None
            raise Exception("Not implemented")
            futures = [
                self.__executor.submit(cam.getVisionMesurement, robotRotation)
                for cam in self.cameras
            ]

    def execute(self):
        pass
