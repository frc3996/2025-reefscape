# import math
# from typing import final
# 
# import ntcore
# import wpilib
# import wpimath
# import wpimath.filter
# import wpimath.geometry
# import wpimath.kinematics
# from magicbot import StateMachine
# from magicbot.state_machine import StateRef, state, timed_state
# 
# from components.swervedrive import SwerveDrive
# 
# 
# @final
# class AngularMaxVelocity(StateMachine):
#     """
#     An autonomous state machine that generates and follows a trajectory.
#     """
# 
#     MODE_NAME = "AngularMaxVelocity"
#     DEFAULT = False  # Only run when explicitly selected.
# 
#     # Inject the SwerveDrive subsystem.
#     drivetrain: SwerveDrive
#     omega: float = 0
# 
#     def setup(self) -> None:
#         super().__init__()
# 
#         nt = ntcore.NetworkTableInstance.getDefault()
#         self.maxAngularVelocityPub = nt.getFloatTopic(
#             f"/AdvantageScope/SysID/MaxAngularVelocity",
#         ).publish()
# 
#     @state(first=True)
#     def set_swerve(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(-45)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(45)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(45)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(-45)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
#         self.next_state("accelerate")
#         self.limiter = wpimath.filter.SlewRateLimiter(1)
# 
#     @state()
#     def accelerate(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(135)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(45)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(45)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(-45)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
# 
#         volts = self.limiter.calculate(wpilib.RobotController.getBatteryVoltage())
#         self.drivetrain.frontLeft.driveMotor.setVoltage(volts)
#         self.drivetrain.frontRight.driveMotor.setVoltage(volts)
#         self.drivetrain.backLeft.driveMotor.setVoltage(volts)
#         self.drivetrain.backRight.driveMotor.setVoltage(volts)
# 
#         omega = self.drivetrain.getChassisSpeeds().omega
#         if omega > self.omega:
#             self.omega = omega
#             self.maxAngularVelocityPub.set(omega)
# 
# 
# @final
# class MaxVelocity(StateMachine):
#     """
#     An autonomous state machine that generates and follows a trajectory.
#     """
# 
#     MODE_NAME = "MaxVelocity"
#     DEFAULT = False  # Only run when explicitly selected.
# 
#     # Inject the SwerveDrive subsystem.
#     drivetrain: SwerveDrive
#     velocity: float = 0
# 
#     def setup(self) -> None:
#         super().__init__()
# 
#         nt = ntcore.NetworkTableInstance.getDefault()
#         self.maxVelocityPub = nt.getFloatTopic(
#             f"/AdvantageScope/SysID/maxVelocity",
#         ).publish()
# 
#     @state(first=True)
#     def set_swerve(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
#         self.next_state("accelerate")
#         self.limiter = wpimath.filter.SlewRateLimiter(1)
# 
#     @state()
#     def accelerate(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
# 
#         volts = self.limiter.calculate(wpilib.RobotController.getBatteryVoltage())
#         self.drivetrain.frontLeft.driveMotor.setVoltage(-volts)
#         self.drivetrain.frontRight.driveMotor.setVoltage(volts)
#         self.drivetrain.backLeft.driveMotor.setVoltage(-volts)
#         self.drivetrain.backRight.driveMotor.setVoltage(volts)
# 
#         velocity = math.hypot(
#             self.drivetrain.getChassisSpeeds().vx, self.drivetrain.getChassisSpeeds().vy
#         )
#         if velocity > self.velocity:
#             self.velocity = velocity
#             self.maxVelocityPub.set(velocity)
# 
# 
# @final
# class MaxAccel(StateMachine):
#     """
#     An autonomous state machine that generates and follows a trajectory.
#     """
# 
#     MODE_NAME = "MaxAccel"
#     DEFAULT = False  # Only run when explicitly selected.
# 
#     # Inject the SwerveDrive subsystem.
#     drivetrain: SwerveDrive
#     accel: float = 0
#     dt: float
# 
#     def setup(self) -> None:
#         super().__init__()
# 
#         nt = ntcore.NetworkTableInstance.getDefault()
#         self.maxAccelPub = nt.getFloatTopic(
#             f"/AdvantageScope/SysID/maxAccel",
#         ).publish()
# 
#     @timed_state(first=True, duration=5, next_state="accelerate")
#     def set_swerve(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
# 
#         self.speed = math.hypot(
#             self.drivetrain.getChassisSpeeds().vx, self.drivetrain.getChassisSpeeds().vy
#         )
# 
#     @timed_state(duration=1)
#     def accelerate(self):
#         swerveModuleStates = (
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#             wpimath.kinematics.SwerveModuleState(
#                 0, wpimath.geometry.Rotation2d.fromDegrees(0)
#             ),
#         )
#         self.drivetrain.frontLeft.setDesiredState(swerveModuleStates[0])
#         self.drivetrain.frontRight.setDesiredState(swerveModuleStates[1])
#         self.drivetrain.backLeft.setDesiredState(swerveModuleStates[2])
#         self.drivetrain.backRight.setDesiredState(swerveModuleStates[3])
# 
#         volts = wpilib.RobotController.getBatteryVoltage()
#         self.drivetrain.frontLeft.driveMotor.setVoltage(-volts)
#         self.drivetrain.frontRight.driveMotor.setVoltage(volts)
#         self.drivetrain.backLeft.driveMotor.setVoltage(-volts)
#         self.drivetrain.backRight.driveMotor.setVoltage(volts)
# 
#         speed = math.hypot(
#             self.drivetrain.getChassisSpeeds().vx, self.drivetrain.getChassisSpeeds().vy
#         )
#         accel = speed - self.speed / self.dt
#         self.speed = speed
# 
#         if accel > self.accel:
#             self.accel = accel
#             self.maxAccelPub.set(accel)
