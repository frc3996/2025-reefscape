import os

import wpimath
import wpimath.units
from magicbot import feedback
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose2d, Rotation2d

import common.tools
from components.reefscape import REEFSCAPE, CagePositionKeys, Position
from components.rikistick import RikiStick
from components.swervedrive import SwerveDrive

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)


def position_to_pose2d(position: Position) -> Pose2d:
    return Pose2d(position["x"], position["y"], Rotation2d.fromDegrees(position["rot"]))


def find_closest_pose(target: Pose2d, poses: list[Pose2d]) -> Pose2d:
    return min(poses, key=lambda p: p.translation().distance(target.translation()))


class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive
    rikiStick: RikiStick

    def __init__(self):
        super().__init__(apriltagsLayoutPath)

    @feedback
    def redPositions(self) -> list[Pose2d]:
        """Just to display things"""
        reef = list([position_to_pose2d(x) for x in REEFSCAPE["red"]["reef"].values()])
        corals_left = list(
            [position_to_pose2d(x) for x in REEFSCAPE["red"]["coral"]["left"].values()]
        )
        corals_right = list(
            [position_to_pose2d(x) for x in REEFSCAPE["red"]["coral"]["right"].values()]
        )
        cage = list([position_to_pose2d(x) for x in REEFSCAPE["red"]["cage"].values()])
        return reef + corals_left + corals_right + cage

    @feedback
    def blueBranches(self) -> list[Pose2d]:
        """Just to display things"""
        reef = list([position_to_pose2d(x) for x in REEFSCAPE["blue"]["reef"].values()])
        corals_left = list(
            [position_to_pose2d(x) for x in REEFSCAPE["blue"]["coral"]["left"].values()]
        )
        corals_right = list(
            [
                position_to_pose2d(x)
                for x in REEFSCAPE["blue"]["coral"]["right"].values()
            ]
        )
        cage = list([position_to_pose2d(x) for x in REEFSCAPE["blue"]["cage"].values()])
        return reef + corals_left + corals_right + cage

    def getSide(self) -> str:
        if common.tools.is_red():
            return "red"
        elif common.tools.is_blue():
            return "blue"
        else:
            raise Exception("Pick a side")

    def getCoralPosition(self) -> Pose2d:
        positions: list[Pose2d] = list(
            [
                position_to_pose2d(x)
                for x in REEFSCAPE[self.getSide()]["coral"][
                    self.rikiStick.coralStation
                ].values()
            ]
        )
        return find_closest_pose(self.drivetrain.getPose(), positions)

    def getCagePosition(self) -> Pose2d | None:
        if self.rikiStick.cagePosition == CagePositionKeys.NONE:
            return None
        print("GOT CAGE")
        return position_to_pose2d(
            REEFSCAPE[self.getSide()]["cage"][self.rikiStick.cagePosition]
        )

    def getReefPosition(self) -> Pose2d:
        position: Position = REEFSCAPE[self.getSide()]["reef"][
            self.rikiStick.reefPosition
        ]
        return position_to_pose2d(position)

    def execute(self):
        pass
