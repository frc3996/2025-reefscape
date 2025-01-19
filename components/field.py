"""
Thank you to the FROG team

https://github.com/FROG3160/2023-charged-up/blob/445c4d6225902a0fd2359cf92854632d6f3e2aaa/roborio-src/components/vision.py

"""
import logging
import os

import wpilib
import wpimath
import wpimath.units
from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Transform3d, Translation3d
from wpimath.units import inchesToMeters

from common import tools
from components.swervedrive import SwerveDrive

apriltagsFilename = r"2025-reefscape.json"
# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)

X_CHANGE = 1.27257
Y_CHANGE = inchesToMeters(22)
Z_CHANGE = 0.462788

RED_ALLIANCE = wpilib.DriverStation.Alliance.kRed
BLUE_ALLIANCE = wpilib.DriverStation.Alliance.kBlue

botPositionsFromTag = (
    Transform3d(Translation3d(X_CHANGE, -Y_CHANGE, -Z_CHANGE), Rotation3d(0, 0, 0)),
    Transform3d(Translation3d(X_CHANGE, 0, -Z_CHANGE), Rotation3d(0, 0, 0)),
    Transform3d(Translation3d(X_CHANGE, Y_CHANGE, -Z_CHANGE), Rotation3d(0, 0, 0)),
)

# tag list ordered by view from driverstation of the alliance color
# ( rightTagID, MiddleTagID, LeftTagID)
blueTagList = (8, 7, 6)
redTagList = (3, 2, 1)

blueGridPositions = {
    1: (8, 1),
    2: (8, 2),
    3: (8, 3),
    4: (7, 1),
    5: (7, 2),
    6: (7, 3),
    7: (6, 1),
    8: (6, 2),
    9: (6, 3),
}

redGridPositions = {
    1: (3, 1),
    2: (3, 2),
    3: (3, 3),
    4: (2, 1),
    5: (2, 2),
    6: (2, 3),
    7: (1, 1),
    8: (1, 2),
    9: (1, 3),
}


def getAlliance():
    return wpilib.DriverStation.getAlliance()


class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive

    def __init__(self):
        self.logger = logging.getLogger("FieldLayout")
        super().__init__(apriltagsLayoutPath)
        # set layout to be specific to the alliance end
        self.alliance = wpilib.DriverStation.Alliance.kBlue
        self.gridPositions = blueGridPositions

    def getTagtoRobotTransform(
        self, field_pose: Pose3d, tagID: int
    ) -> Transform3d | None:
        tag_pose = self.getTagPose(tagID)
        if tag_pose is None:
            return None
        return field_pose - tag_pose

    # get position
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

    # def getPosition(self, position: int) -> Pose3d | None:
    #     """Returns the field pose for the robot to use to be in front
    #     of the given grid position
    #
    #     Args:
    #         position (int): Grid position, from 1 to 9 with 1 being the furthest right
    #
    #     Returns:
    #         Pose3d: Field pose
    #     """
    #     return self.getTagRelativePosition(*self.gridPositions[position])

    def getSpeakerRelativePosition(self, z_offset=0) -> Transform3d | None:
        AMP_HEIGHT_LOW = wpimath.units.feetToMeters(6.5)  # 6'6
        AMP_HEIGHT_HIGH = wpimath.units.feetToMeters(6.90626)  # 6'10 7/8
        # Tag under the Speaker
        tag_pose = self.getTagRelativePosition(4 if tools.is_red() else 7)
        if tag_pose is None:
            return None

        # Aim in front to better adjust angle
        if tools.is_red():
            tag_pose = Transform3d(tag_pose.x, tag_pose.y, tag_pose.z, tag_pose.rotation())
        else:
            tag_pose = Transform3d(tag_pose.x, tag_pose.y, tag_pose.z, tag_pose.rotation())
        # Adjust it for the speaker
        speaker_pose = Transform3d(
            tag_pose.x,
            tag_pose.y,
            wpimath.units.meters(
                AMP_HEIGHT_LOW + (AMP_HEIGHT_HIGH - AMP_HEIGHT_LOW) / 2
            )
            - z_offset,
            Rotation3d(0, 0, 0),
        )
        return speaker_pose

    # set alliance/change origin
    def syncAlliance(self, alliance=getAlliance()):
        self.logger.info(f"FROGFieldLayout.setAlliance() called with {alliance}")
        if alliance == RED_ALLIANCE:
            self.setOrigin(self.OriginPosition.kRedAllianceWallRightSide)
            self.alliance = RED_ALLIANCE
            self.gridPositions = redGridPositions
        elif alliance == BLUE_ALLIANCE:
            self.setOrigin(self.OriginPosition.kBlueAllianceWallRightSide)
            self.alliance = BLUE_ALLIANCE
            self.gridPositions = blueGridPositions

    def execute(self):
        pass
