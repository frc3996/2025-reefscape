import math

from phoenix6 import utils
from wpimath.geometry import Pose2d, Rotation2d
from common.limelight_helpers import PoseEstimate, LimelightHelpers

class LimeLightVision:
    def __init__(self, name : str):
        self.__cameraName = name

    def light_pipeline(self):
        LimelightHelpers.set_LED_to_pipeline_control(self.__cameraName)

    def light_off(self):
        LimelightHelpers.set_LED_to_force_off(self.__cameraName)

    def light_blink(self):
        LimelightHelpers.set_LED_to_force_blink(self.__cameraName)

    def light_on(self):
        LimelightHelpers.set_LED_to_force_on(self.__cameraName)

    def execute(self):
        pass

    def getVisionMesurement(
        self, robotOrientation : Rotation2d,
    ) -> tuple[Pose2d, float, tuple[float, float, float]] | None:

        assert self.__cameraName != ""
        LimelightHelpers.set_robot_orientation(self.__cameraName, robotOrientation.degrees(), 0, 0, 0, 0, 0)
        # LimelightHelpers.set_imu_mode(self.__cameraName, 0)

        # Getting blue megatag2 pose, both teams are in blue-team space
        poseEstimate = LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(self.__cameraName)
        if poseEstimate is None:
            # print(self.__cameraName + ": no pose estimate")
            return None
        elif poseEstimate.tag_count == 0:
            # print(self.__cameraName + ": pose estimate with no tags")
            return None
        else:
            return (
                poseEstimate.pose,
                utils.fpga_to_current_time(poseEstimate.timestamp_seconds), # TODO nécessaire??
                self._get_dynamic_std_devs(poseEstimate),
            )

    @staticmethod
    def _get_dynamic_std_devs(pose: PoseEstimate) -> tuple[float, float, float]:
        """ Computes dynamic standard deviations based on tag count and distance. """
        if pose.tag_count == 0:
            return 0.7, 0.7, 0.7

        avg_dist = sum(f.dist_to_camera for f in pose.raw_fiducials) / pose.tag_count
        factor = 1 + (avg_dist ** 2 / 30)

        return 0.7 * factor, 0.7 * factor, math.inf if pose.is_megatag_2 else (0.7 * factor)
