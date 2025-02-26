###################################################################################
# MIT License
#
# Copyright (c) PhotonVision
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
###################################################################################


import math
from typing import final

import ntcore
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
from magicbot import will_reset_to
from pathplannerlib.commands import DriveFeedforwards, Pose2d
from pathplannerlib.trajectory import SwerveModuleState

import constants
from components.gyro import Gyro

from .swervemodule import SwerveModule

kMaxSpeed = 5.5  # 3 meters per second
kMaxAccel = 0.869  # 3 meters per second squared
kMaxAngularSpeed = 4.931  # rad/sec

kInitialPose = wpimath.geometry.Pose2d(
    wpimath.geometry.Translation2d(1.0, 1.0),
    wpimath.geometry.Rotation2d.fromDegrees(0.0),
)


@final
class SwerveDrive:
    """
    Represents a swerve drive style drivetrain.
    """

    gyro: Gyro
    dt: float

    driverChassisSpeeds = will_reset_to(wpimath.kinematics.ChassisSpeeds())
    # autoChassisSpeeds = will_reset_to(wpimath.kinematics.ChassisSpeeds())
    autoChassisSpeeds = wpimath.kinematics.ChassisSpeeds()
    autoFeedforwards: DriveFeedforwards | None = None

    def setup(self) -> None:
        self.frontLeftLocation = wpimath.geometry.Translation2d(0.381, 0.381)
        self.frontRightLocation = wpimath.geometry.Translation2d(0.381, -0.381)
        self.backLeftLocation = wpimath.geometry.Translation2d(-0.381, 0.381)
        self.backRightLocation = wpimath.geometry.Translation2d(-0.381, -0.381)

        self.frontLeft = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_FL,
            constants.CANIds.SWERVE_ROTATE_FL,
            constants.CANIds.SWERVE_CANCODER_FL,
            1,
            rotation_zero=193,
            inverted=False,
        )
        self.frontRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_FR,
            constants.CANIds.SWERVE_ROTATE_FR,
            constants.CANIds.SWERVE_CANCODER_FR,
            2,
            rotation_zero=76,
            inverted=False,
        )
        self.backLeft = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RL,
            constants.CANIds.SWERVE_ROTATE_RL,
            constants.CANIds.SWERVE_CANCODER_RL,
            3,
            rotation_zero=216,
            inverted=False,
        )
        self.backRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RR,
            constants.CANIds.SWERVE_ROTATE_RR,
            constants.CANIds.SWERVE_CANCODER_RR,
            4,
            rotation_zero=318,
            inverted=False,
        )

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.poseEst = wpimath.estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            kInitialPose,
        )

        self.gyro.reset()

        # LOG
        nt = ntcore.NetworkTableInstance.getDefault()
        self.swerveModuleDesiredStatePub = nt.getStructArrayTopic(
            f"/AdvantageScope/DesiredSwerveModuleStates",
            wpimath.kinematics.SwerveModuleState,
        ).publish()
        self.swerveModuleStatePub = nt.getStructArrayTopic(
            f"/AdvantageScope/SwerveModuleState", wpimath.kinematics.SwerveModuleState
        ).publish()

        self.estimatedPositionPub = nt.getStructTopic(
            f"/AdvantageScope/EstimatedPosition", wpimath.geometry.Pose2d
        ).publish()

        self.swerveDesiredChassisSpeedsPub = nt.getStructTopic(
            f"/AdvantageScope/DesiredChassisSpeed", wpimath.kinematics.ChassisSpeeds
        ).publish()
        self.swerveModuleChassisSpeedsPub = nt.getStructTopic(
            f"/AdvantageScope/ChassisSpeed", wpimath.kinematics.ChassisSpeeds
        ).publish()

        self.desiredChassisSpeeds = wpimath.kinematics.ChassisSpeeds()
        self.desiredSwerveModuleState: tuple[
            SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState
        ] = (
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
            SwerveModuleState(),
        )

    def getPose(self) -> Pose2d:
        return self.poseEst.getEstimatedPosition()

    def drive_auto(
        self,
        chassisSpeeds: wpimath.kinematics.ChassisSpeeds,
        driveFeedforwards: DriveFeedforwards | None = None,
    ):
        self.autoChassisSpeeds = chassisSpeeds
        self.autoFeedforwards = driveFeedforwards

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if fieldRelative:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rot, self.gyro.getRotation2d()
            )
        else:
            chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)

        self.driverChassisSpeeds = chassisSpeeds

        self.apply()

    def apply(self):
        # Merge all the sources
        # Compute the magnitude of the driver input
        weightedX = min(abs(self.driverChassisSpeeds.vx) / kMaxSpeed, 1.0)
        weightedY = min(abs(self.driverChassisSpeeds.vy) / kMaxSpeed, 1.0)
        weightedRot = min(abs(self.driverChassisSpeeds.omega) / kMaxAngularSpeed, 1.0)

        # Blend the two sets of speeds
        blendedX = ((1 - weightedX) * self.autoChassisSpeeds.vx) + (
            weightedX * self.driverChassisSpeeds.vx
        )

        blendedY = ((1 - weightedY) * self.autoChassisSpeeds.vy) + (
            weightedY * self.driverChassisSpeeds.vy
        )

        blendedRot: float = ((1 - weightedRot) * self.autoChassisSpeeds.omega) + (
            weightedRot * self.driverChassisSpeeds.omega
        )

        self.autoChassisSpeeds.vx = 0
        self.autoChassisSpeeds.vy = 0
        self.autoChassisSpeeds.omega = 0

        chassisSpeeds = wpimath.kinematics.ChassisSpeeds(blendedX, blendedY, blendedRot)

        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                chassisSpeeds,
                self.dt,
            )
        )

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        new_speed = self.kinematics.toChassisSpeeds(swerveModuleStates)
        if new_speed == wpimath.kinematics.ChassisSpeeds():
            swerveModuleStates = (
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(-45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(-45)
                ),
            )

        self.frontLeft.setDesiredState(
            swerveModuleStates[0],
            (
                self.autoFeedforwards.accelerationsMPS[0]
                if self.autoFeedforwards
                else None
            ),
        )
        self.frontRight.setDesiredState(
            swerveModuleStates[1],
            (
                self.autoFeedforwards.accelerationsMPS[1]
                if self.autoFeedforwards
                else None
            ),
        )
        self.backLeft.setDesiredState(
            swerveModuleStates[2],
            (
                self.autoFeedforwards.accelerationsMPS[2]
                if self.autoFeedforwards
                else None
            ),
        )
        self.backRight.setDesiredState(
            swerveModuleStates[3],
            (
                self.autoFeedforwards.accelerationsMPS[3]
                if self.autoFeedforwards
                else None
            ),
        )

        # LOG
        self.desiredSwerveModuleState = swerveModuleStates
        self.desiredChassisSpeeds = self.kinematics.toChassisSpeeds(swerveModuleStates)

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        _ = self.poseEst.update(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def addVisionPoseEstimate(
        self, pose: wpimath.geometry.Pose3d, timestamp: float
    ) -> None:
        self.poseEst.addVisionMeasurement(pose.toPose2d(), timestamp)

    def resetPose(self, pose: wpimath.geometry.Pose2d = kInitialPose) -> None:
        self.poseEst.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose,
        )

    def getModuleStates(self) -> list[wpimath.kinematics.SwerveModuleState]:
        return [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState(),
        ]

    def getModulePoses(self) -> list[wpimath.geometry.Pose2d]:
        p = self.poseEst.getEstimatedPosition()
        flTrans = wpimath.geometry.Transform2d(
            self.frontLeftLocation, self.frontLeft.getAbsoluteHeading()
        )
        frTrans = wpimath.geometry.Transform2d(
            self.frontRightLocation, self.frontRight.getAbsoluteHeading()
        )
        blTrans = wpimath.geometry.Transform2d(
            self.backLeftLocation, self.backLeft.getAbsoluteHeading()
        )
        brTrans = wpimath.geometry.Transform2d(
            self.backRightLocation, self.backRight.getAbsoluteHeading()
        )
        return [
            p.transformBy(flTrans),
            p.transformBy(frTrans),
            p.transformBy(blTrans),
            p.transformBy(brTrans),
        ]

    def getChassisSpeeds(self) -> wpimath.kinematics.ChassisSpeeds:
        return self.kinematics.toChassisSpeeds(self.getModuleStates())

    def getVelocity(self) -> wpimath.units.meters_per_second:
        chassisSpeeds = self.getChassisSpeeds()
        return math.sqrt(chassisSpeeds.vx**2 + chassisSpeeds.vy**2)

    def getAngularVelocity(self) -> wpimath.units.radians_per_second_squared:
        chassisSpeeds = self.getChassisSpeeds()
        return chassisSpeeds.omega

    def log(self):
        # The pose
        self.estimatedPositionPub.set(self.poseEst.getEstimatedPosition())

        # The current module state
        self.swerveModuleStatePub.set(self.getModuleStates())
        # The desired module state
        self.swerveModuleDesiredStatePub.set(list(self.desiredSwerveModuleState))

        # The current chassisSpeeds
        self.swerveModuleChassisSpeedsPub.set(self.getChassisSpeeds())
        # The desired chassisSpeeds
        self.swerveDesiredChassisSpeedsPub.set(self.desiredChassisSpeeds)

        self.frontLeft.log()
        self.frontRight.log()
        self.backLeft.log()
        self.backRight.log()

    def simulationPeriodic(self):
        self.frontLeft.simulationPeriodic()
        self.frontRight.simulationPeriodic()
        self.backLeft.simulationPeriodic()
        self.backRight.simulationPeriodic()

    def execute(self):
        # We apply in the drive method, so we can characterize
        # self.apply()
        pass
