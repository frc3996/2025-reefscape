import math
from typing import final

from magicbot import feedback, tunable
from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d

from common.limelight_helpers import LimelightHelpers, PoseEstimate


@final
class LimeLightVision:
    stddev_xy = tunable(0.2)
    stddev_rot = tunable(0.7)

    def __init__(self, name: str):
        self.cameraName = name
        self.visionPose = Pose2d()

    def light_pipeline(self):
        LimelightHelpers.set_LED_to_pipeline_control(self.cameraName)

    def light_off(self):
        LimelightHelpers.set_LED_to_force_off(self.cameraName)

    def light_blink(self):
        LimelightHelpers.set_LED_to_force_blink(self.cameraName)

    def light_on(self):
        LimelightHelpers.set_LED_to_force_on(self.cameraName)

    def execute(self):
        pass

    def getVisionMesurementMT1(self):
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
                utils.fpga_to_current_time(
                    poseEstimate.timestamp_seconds
                ),  # TODO nécessaire??
                self._get_dynamic_std_devs(poseEstimate),
            )

    @feedback
    def getVisionPosition(self):
        return self.visionPose

    def getVisionMesurement(
        self,
        robotOrientation: Rotation2d,
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
            self.visionPose = poseEstimate.pose
            return (
                poseEstimate.pose,
                utils.fpga_to_current_time(
                    poseEstimate.timestamp_seconds
                ),  # TODO nécessaire??
                self._get_dynamic_std_devs(poseEstimate),
            )

    def _get_dynamic_std_devs(self, pose: PoseEstimate) -> tuple[float, float, float]:
        """Computes dynamic standard deviations based on tag count and distance."""
        if pose.tag_count == 0:
            return 0.7, 0.7, 0.7

        avg_dist = sum(f.dist_to_camera for f in pose.raw_fiducials) / pose.tag_count
        factor = 1 + (avg_dist**2 / 30)

        return (
            self.stddev_xy * factor,
            self.stddev_xy * factor,
            math.inf if pose.is_megatag_2 else (self.stddev_rot * factor),
        )

    def on_enable(self):
        pass

    def execute(self):
        pass
