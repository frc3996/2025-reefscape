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
import wpilib
import wpilib.simulation
import wpimath.estimator
import wpimath.geometry
import wpimath.kinematics
import wpimath.units
from magicbot import will_reset_to

import constants
from components.gyro import Gyro

from .swervemodule import SwerveModule

kMaxSpeed = 3.0  # 3 meters per second
kMaxAccel = 3.0  # 3 meters per second squared
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

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

    chassisSpeeds = will_reset_to(wpimath.kinematics.ChassisSpeeds())

    def setup(self) -> None:
        # For publishing
        nt = ntcore.NetworkTableInstance.getDefault()
        self.swerveModuleStatePub = nt.getStructArrayTopic(
            f"/AdvantageScope/SwerveModulesState", wpimath.kinematics.SwerveModuleState
        ).publish()
        self.estimatedPositionPub = nt.getStructTopic(
            f"/AdvantageScope/EstimatedPosition", wpimath.geometry.Pose2d
        ).publish()

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
        )
        self.frontRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_FR,
            constants.CANIds.SWERVE_ROTATE_FR,
            constants.CANIds.SWERVE_CANCODER_FR,
            2,
            rotation_zero=76,
        )
        self.backLeft = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RL,
            constants.CANIds.SWERVE_ROTATE_RL,
            constants.CANIds.SWERVE_CANCODER_RL,
            3,
            rotation_zero=216,
        )
        self.backRight = SwerveModule(
            constants.CANIds.SWERVE_DRIVE_RR,
            constants.CANIds.SWERVE_ROTATE_RR,
            constants.CANIds.SWERVE_CANCODER_RR,
            4,
            rotation_zero=318,
        )

        self.debugField = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Drivetrain Debug", self.debugField)

        # self.gyro = Gyro()
        # self.gyro = wpilib.AnalogGyro(0)
        # self.simGyro = wpilib.simulation.AnalogGyroSim(self.gyro)

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

        self.targetChassisSpeeds = wpimath.kinematics.ChassisSpeeds()

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        chassisSpeeds: wpimath.kinematics.ChassisSpeeds | None = None,
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param periodSeconds: Time
        """
        if not chassisSpeeds:
            if fieldRelative:
                chassisSpeeds = (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
                    )
                )
            else:
                chassisSpeeds = wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)

        # Merge all the sources
        self.chassisSpeeds += chassisSpeeds

        # No speed? LOCK THE WHEELS
        if chassisSpeeds == wpimath.kinematics.ChassisSpeeds():
            swerveModuleStates = (
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(-45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(-45)
                ),
                wpimath.kinematics.SwerveModuleState(
                    0, wpimath.geometry.Rotation2d.fromDegrees(45)
                ),
            )
        else:
            swerveModuleStates = self.kinematics.toSwerveModuleStates(
                wpimath.kinematics.ChassisSpeeds.discretize(
                    self.chassisSpeeds,
                    self.dt,
                )
            )

        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )

        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        self.targetChassisSpeeds = self.kinematics.toChassisSpeeds(swerveModuleStates)

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

    def resetPose(self) -> None:
        self.poseEst.resetPosition(
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            kInitialPose,
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

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.poseEst.getEstimatedPosition()

    def log(self):
        self.estimatedPositionPub.set(self.poseEst.getEstimatedPosition())
        self.swerveModuleStatePub.set(self.getModuleStates())

        # table = "Drive/"
        #
        # pose = self.poseEst.getEstimatedPosition()
        # wpilib.SmartDashboard.putNumber(table + "X", pose.X())
        # wpilib.SmartDashboard.putNumber(table + "Y", pose.Y())
        # wpilib.SmartDashboard.putNumber(table + "Heading", pose.rotation().degrees())
        #
        # chassisSpeeds = self.getChassisSpeeds()
        # wpilib.SmartDashboard.putNumber(table + "VX", chassisSpeeds.vx)
        # wpilib.SmartDashboard.putNumber(table + "VY", chassisSpeeds.vy)
        # wpilib.SmartDashboard.putNumber(
        #     table + "Omega Degrees", chassisSpeeds.omega_dps
        # )
        #
        # wpilib.SmartDashboard.putNumber(
        #     table + "Target VX", self.targetChassisSpeeds.vx
        # )
        # wpilib.SmartDashboard.putNumber(
        #     table + "Target VY", self.targetChassisSpeeds.vy
        # )
        # wpilib.SmartDashboard.putNumber(
        #     table + "Target Omega Degrees", self.targetChassisSpeeds.omega_dps
        # )
        #
        self.frontLeft.log()
        self.frontRight.log()
        self.backLeft.log()
        self.backRight.log()
        #
        # self.debugField.getRobotObject().setPose(self.poseEst.getEstimatedPosition())
        # self.debugField.getObject("SwerveModules").setPoses(self.getModulePoses())

    def simulationPeriodic(self):
        self.frontLeft.simulationPeriodic()
        self.frontRight.simulationPeriodic()
        self.backLeft.simulationPeriodic()
        self.backRight.simulationPeriodic()
        # self.simGyro.setRate(-1.0 * self.getChassisSpeeds().omega_dps)
        # self.simGyro.setAngle(self.simGyro.getAngle() + self.simGyro.getRate() * 0.02)

    def execute(self):
        pass
