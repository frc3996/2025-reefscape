"""
X:      The component of speed in the x direction relative to the field.
        Positive x is away from your alliance wall.
Y:      The component of speed in the y direction relative to the field.
        Positive y is to your left when standing behind the alliance wall.
Angle:  The angle of the robot as measured by a gyroscope. The robot's angle
        is considered to be zero when it is facing directly away from
        your alliance station wall. Remember that this should be CCW positive.
"""

import math
from dataclasses import dataclass

import magicbot
import ntcore
import pathplannerlib.telemetry
from magicbot import feedback
from navx import AHRS
from wpimath import (controller, estimator, filter, geometry, kinematics,
                     trajectory, units)
from wpimath.geometry import Rotation2d

import constants
from common import tools
from components import swervemodule


@dataclass
class SwerveDriveConfig:
    base_width: float
    base_length: float


class SwerveDrive:
    # Modules injectés depuis le code de ../robot.py
    cfg: SwerveDriveConfig
    is_sim: bool
    frontLeftModule: swervemodule.SwerveModule
    frontRightModule: swervemodule.SwerveModule
    rearLeftModule: swervemodule.SwerveModule
    rearRightModule: swervemodule.SwerveModule
    navx: AHRS
    nt: ntcore.NetworkTable

    tmp_speed_factor = magicbot.will_reset_to((1, 1))
    controller_chassis_speed = magicbot.will_reset_to(kinematics.ChassisSpeeds(0, 0, 0))
    auto_chassis_speed = magicbot.will_reset_to(None)
    request_wheel_lock = magicbot.will_reset_to(False)
    __snap_enabled = magicbot.will_reset_to(False)
    permanent_snap = False

    angle_kp = magicbot.tunable(0.002)
    angle_ki = magicbot.tunable(0)
    angle_kd = magicbot.tunable(0.0)
    angle_max_acc = magicbot.tunable(constants.MAX_ANGULAR_VEL)
    angle_max_vel = magicbot.tunable(constants.MAX_ANGULAR_ACCEL)

    def setup(self):
        """
        Appelé après l'injection
        """
        self.permanent_snap = False
        self.navx_offset = Rotation2d()

        self.__snap_angle = Rotation2d(0)
        self.sim_angle = Rotation2d(0)

        self.angle_pid = controller.ProfiledPIDController(
            self.angle_kp,
            self.angle_ki,
            self.angle_kd,
            trajectory.TrapezoidProfile.Constraints(
                constants.MAX_ANGULAR_VEL, constants.MAX_ANGULAR_ACCEL
            ),
        )
        self.angle_pid.enableContinuousInput(-180, 180)
        self.angle_pid.setTolerance(-0.2, 0.2)

        self.kinematics = kinematics.SwerveDrive4Kinematics(
            geometry.Translation2d(self.cfg.base_width / 2, self.cfg.base_length / 2),
            geometry.Translation2d(self.cfg.base_width / 2, -self.cfg.base_length / 2),
            geometry.Translation2d(-self.cfg.base_width / 2, self.cfg.base_length / 2),
            geometry.Translation2d(-self.cfg.base_width / 2, -self.cfg.base_length / 2),
        )

        # self.odometry = kinematics.SwerveDrive4Odometry(
        self.odometry = estimator.SwerveDrive4PoseEstimator(
            self.kinematics,
            geometry.Rotation2d(0),
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            geometry.Pose2d(),
        )
        self.odometry.setVisionMeasurementStdDevs((0.5, 0.5, math.pi / 2))
        self._chassis_speed = kinematics.ChassisSpeeds()

        self.navx_zero()

    def flush(self):
        """
        Cette méthode devrait être appelé pour remettre à zéro le drivetrain.
        Remotes les module swerve à zéro en même temps.
        """
        self.frontLeftModule.flush()
        self.frontRightModule.flush()
        self.rearLeftModule.flush()
        self.rearRightModule.flush()
        # self.angle_pid.reset(trajectory.TrapezoidProfile.State(self.getRotation2d().degrees(), 0))

    def on_enable(self):
        """Automatic robotpy call when robot enter teleop or auto"""
        self.flush()

        self.angle_pid.setPID(
            self.angle_kp,
            self.angle_ki,
            self.angle_kd,
        )
        constraint = trajectory.TrapezoidProfile.Constraints(
            self.angle_max_vel,
            self.angle_max_acc,
        )
        self.angle_pid.setConstraints(constraint)

    def getRotation2d(self):
        return self.get_odometry_pose().rotation()

    def get_chassis_speed(self) -> kinematics.ChassisSpeeds:
        return self._chassis_speed

    def angle_reached(self, acceptable_error=5):
        """Returns if the target angle have been reached"""
        if (
            abs((self.__snap_angle - self.get_odometry_angle()).degrees())
            < acceptable_error
        ):
            return True
        return False

    def get_odometry_angle(self) -> Rotation2d:
        """
        Retourne l'angle absolue basée sur le zéro
        """
        return self.getRotation2d()

    def snap_angle(self, angle: Rotation2d):
        """Écrit l'angle absolue à atteindre de -180 à 180"""
        self.__snap_angle = angle
        self.__snap_enabled = True

    def set_field_relative_automove_value(self, forward, strafe):
        self.auto_chassis_speed = kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            forward, strafe, 0, self.getRotation2d()
        )

    def relative_rotate(self, rotation):
        self.__snap_angle = self.get_odometry_angle() + Rotation2d.fromDegrees(rotation)

    def set_robot_relative_automove_value(self, forward, strafe):
        self.auto_chassis_speed = kinematics.ChassisSpeeds(forward, strafe, 0)

    def set_tmp_speed_factor(self, factor_movement=1.0, factor_rotation=1.0):
        self.tmp_speed_factor = (factor_movement, factor_rotation)

    def set_controller_values(self, forward, strafe, angle_stick_x, angle_stick_y):
        forward = tools.square_input(forward)
        strafe = tools.square_input(strafe)

        if tools.is_red():
            forward = -forward
            strafe = -strafe

        if abs(forward) < constants.LOWER_INPUT_THRESH:
            forward = 0
        if abs(strafe) < constants.LOWER_INPUT_THRESH:
            strafe = 0
        omega = 0
        # omega = tools.square_input(-angle_stick_x)
        # if abs(omega) < constants.LOWER_INPUT_THRESH:
        #     omega = 0

        forward *= constants.MAX_WHEEL_SPEED
        strafe *= constants.MAX_WHEEL_SPEED

        self.controller_chassis_speed = (
            kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, omega, self.getRotation2d()
            )
        )

        # Élimite la zone morte du joystick (petits déplacements)
        if math.sqrt(angle_stick_x**2 + angle_stick_y**2) > 0.25:
            angle = Rotation2d(
                -math.atan2(angle_stick_y, angle_stick_x)
            ) + Rotation2d.fromDegrees(270)

            if tools.is_red():
                angle += Rotation2d.fromDegrees(180)
            self.snap_angle(angle)

    def __calculate_vectors(self):
        """
        Réalise le calcul des angles et vitesses pour chaque swerve module
        en fonction des commandes reçues
        """

        chassis_speed = self.controller_chassis_speed
        if self.auto_chassis_speed is not None:
            chassis_speed += self.auto_chassis_speed

        if self.__snap_enabled or self.permanent_snap:
            omega = self.angle_pid.calculate(
                self.get_odometry_angle().degrees(), self.__snap_angle.degrees()
            )
            omega = max(min(omega, 2), -2)
            if abs(omega) <= 0.002:
                omega = 0
            chassis_speed.omega = -omega
        else:
            self.angle_pid.reset(
                trajectory.TrapezoidProfile.State(self.getRotation2d().degrees(), 0)
            )

        # Ne fais rien si les vecteurs sont trop petits
        if (
            chassis_speed.vx == 0
            and chassis_speed.vy == 0
            and chassis_speed.omega == 0
            and self.request_wheel_lock
        ):
            self.frontLeftModule.setTargetState(
                kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            )
            self.frontRightModule.setTargetState(
                kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(45))
            )
            self.rearLeftModule.setTargetState(
                kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(-45))
            )
            self.rearRightModule.setTargetState(
                kinematics.SwerveModuleState(0, Rotation2d.fromDegrees(45))
            )
            return

        chassis_speed.vx = chassis_speed.vx * self.tmp_speed_factor[0]
        chassis_speed.vy = chassis_speed.vy * self.tmp_speed_factor[0]
        chassis_speed.omega = chassis_speed.omega * self.tmp_speed_factor[1]

        self.sim_angle = self.sim_angle + Rotation2d.fromDegrees(
            chassis_speed.omega * 1 * 20
        )

        chassis_speed = kinematics.ChassisSpeeds.discretize(
            chassis_speed,
            0.02,
        )
        self._chassis_speed = chassis_speed

        swerveModuleStates = self.kinematics.toSwerveModuleStates(chassis_speed)
        kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, constants.MAX_WHEEL_SPEED
        )

        self.frontLeftModule.setTargetState(swerveModuleStates[0])
        self.frontRightModule.setTargetState(swerveModuleStates[1])
        self.rearLeftModule.setTargetState(swerveModuleStates[2])
        self.rearRightModule.setTargetState(swerveModuleStates[3])

    def navx_zero(self):
        self.navx.reset()
        self.navx_update_offset()
        self.__snap_angle = self.get_odometry_angle()
        # self.angle_pid.reset(trajectory.TrapezoidProfile.State(0, 0))

    def navx_update_offset(self):
        self.navx_offset = (
            self.navx.getRotation2d() - self.odometry.getEstimatedPosition().rotation()
        )

    def __updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""

        # Get angle from navx!
        if self.is_sim:
            gyro_angle = self.sim_angle
        else:
            if self.navx.isConnected():
                gyro_angle = self.navx.getRotation2d() + self.navx_offset
            else:
                print("NAVX SHARTED")
                return

        # Add odometry measurements
        self.odometry.update(
            gyro_angle,
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
        )
        pathplannerlib.telemetry.PPLibTelemetry.setCurrentPose(self.get_odometry_pose())

    def get_odometry_pose(self) -> geometry.Pose2d:
        """
        For PathPlannerLib
        Robot pose supplier
        """
        return self.odometry.getEstimatedPosition()

    def resetPose(self, pose: geometry.Pose2d):
        """
        For PathPlannerLib
        Method to reset odometry (will be called if your auto has a starting pose)
        """
        gyro = (
            self.sim_angle
            if self.is_sim
            else (self.navx.getRotation2d() + self.navx_offset)
        )

        self.odometry.resetPosition(
            gyro,
            (
                self.frontLeftModule.getPosition(),
                self.frontRightModule.getPosition(),
                self.rearLeftModule.getPosition(),
                self.rearRightModule.getPosition(),
            ),
            pose,
        )

    def execute(self):
        """
        Calcul et transmet la commande de vitesse et d'angle à chaque swerve module.
        """
        # Calcul des vecteurs
        self.__calculate_vectors()

        # Évaluation de la commande selon le mode d'opération
        self.__updateOdometry()

        self.frontLeftModule.process()
        self.frontRightModule.process()
        self.rearLeftModule.process()
        self.rearRightModule.process()
