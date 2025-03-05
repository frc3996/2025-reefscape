import math

import wpilib
import wpimath
import wpimath.units
from ntcore import (DoubleArraySubscriber, DoubleSubscriber,
                    FloatArraySubscriber, FloatSubscriber, IntegerSubscriber,
                    NetworkTable, NetworkTableInstance, StringSubscriber)
from wpimath.filter import MedianFilter
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Translation3d

from common import tools
from components.field import FieldLayout
from components.swervedrive import SwerveDrive


class LimeLightVision:

    def __init__(self, name="limelight"):
        self.nt: NetworkTable = NetworkTableInstance.getDefault().getTable(name)

        # Register to the limelight topics
        self.tclass: StringSubscriber = self.nt.getStringTopic("tclass").subscribe(
            "None"
        )
        self.ta: FloatSubscriber = self.nt.getFloatTopic("ta").subscribe(0)
        self.tx: FloatSubscriber = self.nt.getFloatTopic("tx").subscribe(-999)
        self.tv: IntegerSubscriber = self.nt.getIntegerTopic("tv").subscribe(0)
        self.pipe: IntegerSubscriber = self.nt.getIntegerTopic("getpipe").subscribe(-1)
        self.cl: FloatSubscriber = self.nt.getFloatTopic("cl").subscribe(0)
        self.tl: FloatSubscriber = self.nt.getFloatTopic("tl").subscribe(0)
        self.tid: IntegerSubscriber = self.nt.getIntegerTopic("tid").subscribe(0)
        self.ts: FloatSubscriber = self.nt.getFloatTopic("ts").subscribe(0)
        # stddevs	doubleArray	MegaTag Standard Deviations
        # [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1Yaw, MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
        self.stddevs: DoubleArraySubscriber = self.nt.getDoubleArrayTopic(
            "stddevs"
        ).subscribe([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])

        self.botpose: FloatArraySubscriber = self.nt.getFloatArrayTopic(
            "botpose"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1, 0, 0, 0, 0])
        self.botpose_wpiblue: FloatArraySubscriber = self.nt.getFloatArrayTopic(
            "botpose_wpiblue"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1, 0, 0, 0, 0])
        self.botpose_wpired: FloatArraySubscriber = self.nt.getFloatArrayTopic(
            "botpose_wpired"
        ).subscribe([-99, -99, -99, 0, 0, 0, -1, 0, 0, 0, 0])

        # create the timer that we can use to the the FPGA timestamp
        self.timer: wpilib.Timer = wpilib.Timer()

        # And a bunch of filters
        self.poseXFilter: MedianFilter = MedianFilter(20)
        self.poseYFilter: MedianFilter = MedianFilter(20)
        self.poseZFilter: MedianFilter = MedianFilter(20)
        self.poseYawFilter: MedianFilter = MedianFilter(20)
        self.__last_update = None
        self.std_devs: list[float] = [0.0, 0.0, 0.0]
        self.filter_pos: list[float] = [0.0, 0.0, 0.0]

    def get_pose(self) -> tuple[Pose3d, float] | None:
        pose3d = self.botpose_to_pose3d(self.botpose.get())
        if pose3d is None:
            return None
        return (*pose3d[:2],)

    def get_alliance_pose(self) -> tuple[Pose3d, float, list[float]] | None:
        if tools.is_red():
            pose3d = self.botpose_to_pose3d(self.botpose_wpired.get())
        else:
            pose3d = self.botpose_to_pose3d(self.botpose_wpiblue.get())

        if pose3d is None:
            return None
        return (*pose3d,)

    def botpose_to_pose3d(
        self, poseArray: list[float]
    ) -> tuple[Pose3d, float, list[float]] | None:
        """Takes limelight array data and creates a Pose3d object for
           robot position and a timestamp reprepresenting the time
           the position was observed.

        Args:
            poseArray (_type_): An array from the limelight network tables.

        Returns:
            tuple[Pose3d, Any]: Returns vision Pose3d and timestamp.
        """
        (
            pX,
            pY,
            pZ,
            pRoll,
            pPitch,
            pYaw,
            msLatency,
            tagCount,
            tagSpan,
            avgTagDist,
            avgTagArea,
        ) = poseArray[:11]
        if tagCount == 0:
            return None
        else:
            return (
                Pose3d(
                    Translation3d(pX, pY, pZ),
                    Rotation3d.fromDegrees(pRoll, pPitch, pYaw),
                ),
                self.timer.getFPGATimestamp() - (msLatency / 1000),
                poseArray,
            )

    def get_std_devs(self):
        return self.std_devs

    def light_pipeline(self):
        _ = self.nt.putNumber("ledMode", 0)

    def light_off(self):
        _ = self.nt.putNumber("ledMode", 1)

    def light_blink(self):
        _ = self.nt.putNumber("ledMode", 2)

    def light_on(self):
        _ = self.nt.putNumber("ledMode", 3)

    def get_filter_pos(self):
        return self.filter_pos

    def execute(self):
        pass

    def getVisionMesurement(
        self,
        drivetrain: SwerveDrive,
        previous_pose: tuple[Pose3d, float, list[float]] | None = None,
        apply: bool = False,
    ):
        # Add vision pose measurements
        vision_pose = self.get_alliance_pose()

        # TODO: This is unused for now, instead try to rely on std deviation\
        if vision_pose is None or vision_pose[0].x == 0 or vision_pose[0].y == 0:
            self.poseXFilter.reset()
            self.poseYFilter.reset()
            self.poseYawFilter.reset()

        if vision_pose and vision_pose[0].x > 0 and vision_pose[0].y > 0:
            """
            To promote stability of the pose estimate and make it robust to bad vision
            data, we recommend only adding vision measurements that are already within
            one meter or so of the current pose estimate.

            Note that the vision measurement standard deviations passed into this
            method will continue to apply to future measurements until a subsequent
            call to SetVisionMeasurementStdDevs() or this method.
            """
            if self.__last_update == vision_pose[0]:
                return

            x = self.poseXFilter.calculate(vision_pose[0].x)
            y = self.poseYFilter.calculate(vision_pose[0].y)
            yaw = self.poseYawFilter.calculate(vision_pose[0].rotation().z)

            # Increase standard deviation depending on the target area
            if self.__last_update is None:
                stddev_xy = 0
                stddev_rot = 0
            else:
                # The default standard deviations of the vision measurements are
                # 0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
                # We're using the limelight target area to increase the std
                # deviations the further we are
                stddev_xy = ((1 - self.ta.get() / 100) ** 4) * 10
                # Use a pretty high std dev as the gyro is much more precise
                stddev_rot = math.pi + ((1 - self.ta.get() / 100) ** 4) * math.pi

            limelight_stddevs = self.stddevs.get()

            self.filter_pos = [x, y, yaw]

            self.__last_update = vision_pose[0]

            if previous_pose is not None:
                if vision_pose[2][9] > previous_pose[2][9]:
                    vision_pose = previous_pose

            if apply:
                drivetrain.addVisionPoseEstimate(
                    vision_pose[0].toPose2d(),
                    vision_pose[1],
                    (
                        limelight_stddevs[0],
                        limelight_stddevs[1],
                        limelight_stddevs[5],
                    ),
                )

            return vision_pose
