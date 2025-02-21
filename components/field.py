import math
import os
from enum import Enum

import wpimath
import wpimath.units
from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import (Pose2d, Pose3d, Rotation2d, Rotation3d,
                              Transform3d, Translation2d)
from wpimath.units import degreesToRadians

from components.swervedrive import SwerveDrive

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)


class CoralStationSide(Enum):
    LEFT = "LEFT"
    RIGHT = "RIGHT"


class CoralStationPickupPosition(Enum):
    P1 = "P1"
    P2 = "P2"
    P3 = "P3"


class ReefBranch(Enum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"
    F = "F"
    G = "G"
    H = "H"
    I = "I"
    J = "J"
    K = "K"
    L = "L"


class Level(Enum):
    L2 = "L2"
    L3 = "L3"
    L4 = "L4"


def getCoralStation():
    # rikistick.coralStation ... convert
    pass


def getReef():
    # rikistick.coralStation ... convert
    pass


FIELD_LENGTH = 17.548
FIELD_WIDTH = 8.052

# Leach (scoring node) heights for each level
coral_heights = {
    "l1": wpimath.units.feetToMeters(2.500),
    "l2": wpimath.units.feetToMeters(5.000),
    "l3": wpimath.units.feetToMeters(7.500),
    "l4": wpimath.units.feetToMeters(1.000),
}

blue_corals = {
    "LEFT": {
        "P1": Pose2d(0.351154, 6.89648, Rotation2d(math.pi * 3 / 4)),
        "P2": Pose2d(0.851154, 7.3964799999999995, Rotation2d(math.pi * 3 / 4)),
        "P3": Pose2d(1.351154, 7.89648, Rotation2d(math.pi * 3 / 4)),
    },
    "RIGHT": {
        "P1": Pose2d(0.451154, 1.15532, Rotation2d(-math.pi * 3 / 4)),
        "P2": Pose2d(0.851154, 0.65532, Rotation2d(-math.pi * 3 / 4)),
        "P3": Pose2d(1.351154, 0.15532, Rotation2d(-math.pi * 3 / 4)),
    },
}

red_corals = {
    "LEFT": {
        "P1": Pose2d(17.197198, 1.15532, Rotation2d(-math.pi / 4)),
        "P2": Pose2d(16.697198, 0.65532, Rotation2d(-math.pi / 4)),
        "P3": Pose2d(16.197198, 0.15532, Rotation2d(-math.pi / 4)),
    },
    "RIGHT": {
        "P1": Pose2d(17.197198, 6.89648, Rotation2d(math.pi / 4)),
        "P2": Pose2d(16.697198, 7.3964799999999995, Rotation2d(math.pi / 4)),
        "P3": Pose2d(16.197198, 7.89648, Rotation2d(math.pi / 4)),
    },
}

blue_branches = {
    "A": Pose2d(3.71, 4.19, Rotation2d(degreesToRadians(0))),
    "B": Pose2d(3.71, 3.862, Rotation2d(degreesToRadians(0))),
    "C": Pose2d(3.939, 3.434, Rotation2d(degreesToRadians(60))),
    "D": Pose2d(4.243, 3.269, Rotation2d(degreesToRadians(60))),
    "E": Pose2d(4.736, 3.272, Rotation2d(degreesToRadians(120))),
    "F": Pose2d(5.02, 3.435, Rotation2d(degreesToRadians(120))),
    "G": Pose2d(5.267, 3.862, Rotation2d(degreesToRadians(180))),
    "H": Pose2d(5.267, 4.19, Rotation2d(degreesToRadians(180))),
    "I": Pose2d(5.02, 4.615, Rotation2d(degreesToRadians(-120))),
    "J": Pose2d(4.737, 4.783, Rotation2d(degreesToRadians(-120))),
    "K": Pose2d(4.782, 4.243, Rotation2d(degreesToRadians(-60))),
    "L": Pose2d(3.959, 4.616, Rotation2d(degreesToRadians(-60))),
}

red_branches = {
    "A": Pose2d(13.838, 4.19, Rotation2d(degreesToRadians(180))),
    "B": Pose2d(13.838, 3.862, Rotation2d(degreesToRadians(180))),
    "C": Pose2d(13.609, 3.434, Rotation2d(degreesToRadians(120))),
    "D": Pose2d(13.305, 3.269, Rotation2d(degreesToRadians(120))),
    "E": Pose2d(12.812, 3.272, Rotation2d(degreesToRadians(60))),
    "F": Pose2d(12.528, 3.435, Rotation2d(degreesToRadians(60))),
    "G": Pose2d(12.281, 3.862, Rotation2d(degreesToRadians(0))),
    "H": Pose2d(12.281, 4.19, Rotation2d(degreesToRadians(0))),
    "I": Pose2d(12.528, 4.615, Rotation2d(degreesToRadians(-60))),
    "J": Pose2d(12.811, 4.783, Rotation2d(degreesToRadians(-60))),
    "K": Pose2d(12.766, 4.243, Rotation2d(degreesToRadians(-120))),
    "L": Pose2d(13.589, 4.616, Rotation2d(degreesToRadians(-120))),
}


class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive

    def __init__(self):
        super().__init__(apriltagsLayoutPath)

    @feedback
    def redBranches(self) -> list[Pose2d]:
        """Just to display things"""
        branches = list(red_branches.values())
        corals_left = list(red_corals["LEFT"].values())
        corals_right = list(red_corals["RIGHT"].values())
        return branches + corals_left + corals_right

    @feedback
    def blueBranches(self) -> list[Pose2d]:
        """Just to display things"""
        branches = list(blue_branches.values())
        corals_left = list(blue_corals["LEFT"].values())
        corals_right = list(blue_corals["RIGHT"].values())
        return branches + corals_left + corals_right

    def getTagRelativePosition(self, tagID: int) -> Transform3d | None:
        """
        Calculate the relative position of the tag
        """
        tag_pose = self.getTagPose(tagID)
        if tag_pose is None:
            return None
        odometry = self.drivetrain.get_odometry_pose()

        # TODO: Maybe implement a pose3d in the swerve, pass in the height??
        odometry_3d = Pose3d(
            odometry.x,
            odometry.y,
            wpimath.units.meters(0),  # Robot is on the floor
            Rotation3d(0, 0, 0),
        )
        return tag_pose - odometry_3d

    def execute(self):
        pass
