import math
import wpilib

from concurrent import futures
from typing import List
from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d
from common.limelight_helpers import PoseEstimate, LimelightHelpers
from components.limelight import LimeLightVision
from components.swervedrive import SwerveDrive
from components.gyro import Gyro

class VisionSystem:

    __cameras : List[LimeLightVision] = []
    __threaded : bool = False
    __executor : futures.ThreadPoolExecutor | None
    drivetrain : SwerveDrive
    gyro: Gyro

    def __init__(self, cameras : List[LimeLightVision], doThreads : bool):
        self.__cameras = cameras
        self.__executor = None
        self.__threaded = doThreads
        if doThreads:
            self.__executor = futures.ThreadPoolExecutor()

    def miseAjour(self):
        if self.gyro.yawSpeed() > 720:
            print("VisionSystem : skipping, yaw rot speed too high")
            return

        # TODO Est-ce qu'on devrait utiliser l'estimé du drivetrain ou le gyro?
        robotRotation = self.drivetrain.getPose().rotation() 
        if self.gyro.ok():
            robotRotation = self.gyro.getRotation2d()

        if not self.__threaded:         
            for cam in self.__cameras:
                ret = cam.getVisionMesurement(robotRotation)
                if ret is not None:
                    pose: Pose2d
                    timestamp: float
                    stddevs: tuple[float, float, float]
                    pose, timestamp, stddevs = ret
                    self.drivetrain.poseEst.addVisionMeasurement(pose, timestamp, stddevs)
        else:
            assert self.__executor is not None
            raise Exception("Not implemented")
            futures = [
                self.__executor.submit(cam.getVisionMesurement, robotRotation)
                for cam in self.__cameras
            ]
