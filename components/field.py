import os

import wpimath
import wpimath.units
from robotpy_apriltag import AprilTagFieldLayout
from wpimath.geometry import Pose3d, Rotation3d, Transform3d

from components.swervedrive import SwerveDrive

# get the dir of THIS file (vision.py), go up one level (..), and use the specified filename
apriltagsFilename = r"2025-reefscape.json"
apriltagsLayoutPath = os.path.join(os.path.dirname(__file__), r"..", apriltagsFilename)


class FieldLayout(AprilTagFieldLayout):
    drivetrain: SwerveDrive

    def __init__(self):
        super().__init__(apriltagsLayoutPath)

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
