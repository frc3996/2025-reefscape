import math
from typing import final

import ntcore
from magicbot import feedback, tunable
from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d

from common.limelight_helpers import LimelightHelpers, PoseEstimate


@final
class LimeLightVision:
    def __init__(self, name: str):
        self.cameraName = name
        self.nt = ntcore.NetworkTableInstance.getDefault()
        self.stddevXY = self.nt.getFloatTopic(
            f"/AdvantageScope/Limelight/stddev_xy",
        ).getEntry(0.01)
        self.stddevXY.set(0.01)

        self.stddevRot = self.nt.getFloatTopic(
            f"/AdvantageScope/Limelight/stddev_rot",
        ).getEntry(0.01)
        self.stddevRot.set(0.01)

    def light_pipeline(self):
        LimelightHelpers.set_LED_to_pipeline_control(self.cameraName)

    def light_off(self):
        LimelightHelpers.set_LED_to_force_off(self.cameraName)

    def light_blink(self):
        LimelightHelpers.set_LED_to_force_blink(self.cameraName)

    def light_on(self):
        LimelightHelpers.set_LED_to_force_on(self.cameraName)

    def setRobotOrientation(self, robotPose: Pose2d):
        LimelightHelpers.set_robot_orientation(
            self.cameraName, robotPose.rotation().degrees(), 0, 0, 0, 0, 0
        )

    def getVisionMesurement(
        self,
    ) -> tuple[Pose2d, float, tuple[float, float, float]] | None:

        assert self.cameraName != ""
        # Getting blue megatag2 pose, both teams are in blue-team space
        poseEstimate = LimelightHelpers.get_botpose_estimate_wpiblue(self.cameraName)
        if poseEstimate is None:
            # print(self.__cameraName + ": no pose estimate")
            return None
        elif poseEstimate.tag_count == 0:
            # print(self.__cameraName + ": pose estimate with no tags")
            return None
        else:
            return (
                poseEstimate.pose,
                utils.fpga_to_current_time(poseEstimate.timestamp_seconds),
                self._get_dynamic_std_devs(poseEstimate),
            )

    def _get_dynamic_std_devs(self, pose: PoseEstimate) -> tuple[float, float, float]:
        """Computes dynamic standard deviations based on tag count and distance."""
        if pose.tag_count == 0:
            return 0.7, 0.7, 0.7

        avg_dist = sum(f.dist_to_camera for f in pose.raw_fiducials) / pose.tag_count
        factor = 1 + (avg_dist**2 / 30)

        stddev_xy = self.stddevXY.get()

        return (
            stddev_xy * factor,
            stddev_xy * factor,
            math.inf if pose.is_megatag_2 else (self.stddevRot.get() * factor),
        )
