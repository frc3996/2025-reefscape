import math
from typing import override

import wpilib
from commands2 import Subsystem
from hal import initialize
from magicbot.state_machine import StateMachine, state
from pathplannerlib.auto import AutoBuilder, FollowPathCommand
from pathplannerlib.commands import PathfindingCommand
from pathplannerlib.config import PIDConstants, RobotConfig
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.path import (GoalEndState, IdealStartingState,
                                 PathConstraints, PathPlannerPath)
from pathplannerlib.pathfinding import Pathfinding
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import degreesToRadians

from components import swervedrive


class ActionPathPlanner(StateMachine):
    drivetrain: swervedrive.SwerveDrive

    class SwerveSubsystem(Subsystem):
        pass

    def setup(self):
        # Do all subsystem initialization here ...

        # Load the RobotConfig from the GUI settings. You should probably
        # store this in your Constants file
        self.robotConfig = RobotConfig.fromGUISettings()

        self.subsystem: Subsystem = self.SwerveSubsystem()

        @staticmethod
        def getEstimatedPosition():
            # Prevent the return of a negative value. It breaks pathplanner
            pos = self.drivetrain.getPose()
            return Pose2d(
                pos.X() if pos.X() > 0 else 0,
                pos.Y() if pos.Y() > 0 else 0,
                pos.rotation(),
            )

        def getChassisSpeeds():
            speed = self.drivetrain.getChassisSpeeds()
            print(f"auto: getChassisSpeeds {speed}")
            return speed

        # Configure the AutoBuilder last
        AutoBuilder.configure(
            # Robot pose supplier
            getEstimatedPosition,
            # Method to reset odometry (will be called if your auto has a
            # starting pose)
            self.drivetrain.resetPose,
            # ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            getChassisSpeeds,
            # Method that will drive the robot given ROBOT RELATIVE
            # ChassisSpeeds. Also outputs individual module feedforwards
            lambda speeds, feedforwards: self.drivetrain.drive_auto(speeds),
            # PPHolonomicController is the built in path following controller
            # for holonomic drive trains
            PPHolonomicDriveController(
                PIDConstants(5.0, 0.0, 0.1),  # Translation PID constants
                PIDConstants(5.0, 0.0, 0.1),  # Rotation PID constants
            ),
            self.robotConfig,  # The robot configuration
            self.shouldFlipPath,  # Supplier to control path flipping based on alliance color
            self.subsystem,  # Reference to this subsystem to set requirements
        )

        # Create the constraints to use while pathfinding
        self.constraints = PathConstraints(
            swervedrive.kMaxSpeed,
            swervedrive.kMaxAccel,
            swervedrive.kMaxAngularSpeed,
            swervedrive.kMaxAngularSpeed / 3,  # A wild guess..
        )
        # You can also use unlimited constraints, only limited by motor torque
        # and nominal battery voltage
        # self.constraints: PathConstraints = PathConstraints.unlimitedConstraints(
        #     wpilib.RobotController.getBatteryVoltage()
        # )

    @staticmethod
    def shouldFlipPath():
        # Boolean supplier that controls when the path will be mirrored for the red alliance
        # This will flip the path being followed to the red side of the field.
        # THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return False
        # return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    @state(first=True)
    def initial(self):
        # Since we are using a holonomic drivetrain, the rotation component of
        # this pose represents the goal holonomic rotation
        targetPose = Pose2d(6, 6, Rotation2d.fromDegrees(180))

        if False:
            self.command: PathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                self.constraints,
            )
        elif False:
            # Create a list of waypoints from poses. Each pose represents one
            # waypoint.
            currentPose = self.drivetrain.getPose()
            waypoints = PathPlannerPath.waypointsFromPoses([currentPose, targetPose])
            # waypoints = PathPlannerPath.waypointsFromPoses(
            #     [
            #         Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            #         Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            #         Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
            #     ]
            # )

            chassisSpeeds = self.drivetrain.getChassisSpeeds()
            currentSpeed = math.hypot(chassisSpeeds.vx, chassisSpeeds.vy)
            # Create the path using the waypoints created above
            path = PathPlannerPath(
                waypoints,
                self.constraints,
                # The ideal starting state, this is only relevant for pre-planned
                # paths, so can be None for on-the-fly paths.
                IdealStartingState(currentSpeed, currentPose.rotation()),
                # None,
                # Goal end state. You can set a holonomic rotation here. If using a
                # differential drivetrain, the rotation will have no effect.
                GoalEndState(0.0, targetPose.rotation()),
                # GoalEndState(0.0, Rotation2d.fromDegrees(-90)),
                is_reversed=False,
            )

            # Prevent the path from being flipped if the coordinates are already correct
            # path.preventFlipping = True

            self.command: PathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                path, self.constraints
            )
        else:
            currentPose = self.drivetrain.getPose()
            waypoints = PathPlannerPath.waypointsFromPoses([currentPose, targetPose])
            chassisSpeeds = self.drivetrain.getChassisSpeeds()
            currentSpeed = math.hypot(chassisSpeeds.vx, chassisSpeeds.vy)

            # Create the path using the waypoints created above
            path = PathPlannerPath(
                waypoints,
                self.constraints,
                # The ideal starting state, this is only relevant for pre-planned
                # paths, so can be None for on-the-fly paths.
                IdealStartingState(currentSpeed, currentPose.rotation()),
                # None,
                # Goal end state. You can set a holonomic rotation here. If using a
                # differential drivetrain, the rotation will have no effect.
                GoalEndState(0.0, targetPose.rotation()),
                # GoalEndState(0.0, Rotation2d.fromDegrees(-90)),
                is_reversed=False,
            )

            # self.command: PathfindingCommand = PathfindingCommand(
            #     self.constraints,
            #     self.drivetrain.getPose,
            #     self.drivetrain.getChassisSpeeds,
            #     lambda speeds, feedforwards: self.drivetrain.drive_auto(speeds),
            #     PPHolonomicDriveController(
            #         PIDConstants(5.0, 0.0, 0.1),  # Translation PID constants
            #         PIDConstants(5.0, 0.0, 0.1),  # Rotation PID constants
            #     ),
            #     self.robotConfig,
            #     lambda: False,
            #     self.subsystem,
            #     target_path=path,
            #     # target_pose=targetPose,
            #     # goal_end_vel=0,
            # )

            self.command: FollowPathCommand = FollowPathCommand(
                path,
                self.drivetrain.getPose,
                self.drivetrain.getChassisSpeeds,
                lambda speeds, feedforwards: self.drivetrain.drive_auto(speeds),
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.0, 0.1),  # Translation PID constants
                    PIDConstants(5.0, 0.0, 0.1),  # Rotation PID constants
                ),
                self.robotConfig,
                lambda: False,
                self.subsystem,
            )

        self.command.initialize()

        self.next_state("exec")

    @state()
    def exec(self):
        self.command.execute()

        if self.command.isFinished():
            print("FINISH")
            self.next_state("finish")

    @state()
    def finish(self, initial_call: bool):
        if initial_call:
            self.command.end(False)

    @override
    def done(self) -> None:
        if self.is_executing:
            print("CANCELING")
            self.command.end(True)
        return super().done()


#
# # Load the path we want to pathfind to and follow
#             path = PathPlannerPath.fromPathFile('Example Human Player Pickup');
#
# # Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
#             constraints = PathConstraints(
#                 3.0, 4.0,
#                 degreesToRadians(540), degreesToRadians(720)
#             )
#
# # Since AutoBuilder is configured, we can use it to build pathfinding commands
#             pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
#                 path,
#                 constraints,
#                 rotation_delay_distance=3.0 # Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
#             )
# @state(first=True)
# def initial(self):
#     # XXX: Probably not necessary
#     self.subsystem.periodic()
#
#     # Create a list of waypoints from poses. Each pose represents one
#     # waypoint.
#     # The rotation component of the pose should be the direction of travel.
#     # Do not use holonomic rotation.
#     waypoints = PathPlannerPath.waypointsFromPoses(
#         [
#             Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
#             Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
#             Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90)),
#         ]
#     )
#     # The constraints for this path.
#     constraints = PathConstraints(3.0, 3.0, 2 * math.pi, 4 * math.pi)
#
#     # You can also use unlimited constraints, only limited by motor torque
#     # and nominal battery voltage
#     # constraints = PathConstraints.unlimitedConstraints(12.0)
#
#     # Create the path using the waypoints created above
#     path = PathPlannerPath(
#         waypoints,
#         constraints,
#         # The ideal starting state, this is only relevant for pre-planned
#         # paths, so can be None for on-the-fly paths.
#         None,
#         # Goal end state. You can set a holonomic rotation here. If using a
#         # differential drivetrain, the rotation will have no effect.
#         GoalEndState(0.0, Rotation2d.fromDegrees(-90)),
#     )
#
#     # Prevent the path from being flipped if the coordinates are already correct
#     path.preventFlipping = True
