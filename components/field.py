import os

import wpimath
import wpimath.units
from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d

import common.tools
from components.rikistick import RikiStick
from components.swervedrive import SwerveDrive
from components.reefscape import Reefscape

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

def find_closest_pose(target: Pose2d, poses: list[Pose2d]) -> Pose2d:
    return min(poses, key=lambda p: p.translation().distance(target.translation()))

class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive
    rikiStick: RikiStick
    reefscape : Reefscape

    def __init__(self):
        super().__init__(apriltagsLayoutPath)

    @feedback
    def redTargets(self) -> list[Pose2d]:
        """Just to display things"""
        return self.reefscape.getAllRedPoints()

    @feedback
    def blueTargets(self) -> list[Pose2d]:
        """Just to display things"""
        return self.reefscape.getAllBluePoints()

    def getCoralPosition(self) -> Pose2d:
        return find_closest_pose(self.drivetrain.getPose(),
                                 self.reefscape.getAllCoralStationSlides(self.rikiStick.getCoralStationTarget()))

    def getCagePosition(self) -> Pose2d | None:
        if self.rikiStick.getCageTarget() == 0:
            return None
        else:
            return self.reefscape.getCage(self.rikiStick.getCageTarget())

    def getReefPosition(self) -> Pose2d:
        return self.reefscape.getReef(self.rikiStick.getReefTarget())

    def execute(self):
        pass
