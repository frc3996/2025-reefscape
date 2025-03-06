import os

import wpimath
import wpimath.units
from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d, Transform2d

import common.tools
from components.reefscape import Reefscape
from components.rikistick import RikiStick
from components.swervedrive import SwerveDrive

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)


class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive
    rikiStick: RikiStick
    reefscape: Reefscape

    def __init__(self):
        super().__init__(apriltagsLayoutPath)

    # @feedback
    # def redTargets(self) -> list[Pose2d]:
    #     """Just to display things"""
    #     return self.reefscape.getAllRedPoints()
    #
    # @feedback
    # def blueTargets(self) -> list[Pose2d]:
    #     """Just to display things"""
    #     return self.reefscape.getAllBluePoints()

    def getCoralPosition(self) -> Pose2d:
        pos = self.reefscape.getClosestCoralStationSlide(
            self.rikiStick.getCoralStationTarget(), self.drivetrain.getPose()
        )
        # TODO: Fix in map
        return pos.rotateBy(Rotation2d.fromDegrees(180))

    def getCagePosition(self) -> Pose2d | None:
        return self.reefscape.getCage(
            1
        )  # TODO la cage choisi par le gamepad avec la killswitch à off?

    def getReefPosition(self) -> Pose2d:
        return self.reefscape.getReef(self.rikiStick.getReefTarget())
